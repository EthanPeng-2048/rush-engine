module;
#include <array>
#include <cmath>
#include <string>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
#include <exstl/errors.hpp>

export module rush.engine.physics;

import rush.engine.core;
import rush.engine.render;
import rush.engine.events;

export namespace apex {


using BodyHandle = uint32_t;
constexpr BodyHandle kInvalidBody = 0;

// Convention for BodyDesc.userData on track surfaces: the pointer carries a
// SurfaceType value cast through uintptr_t — never a real pointer. Ray hits
// and contact events report it back so the vehicle can pick grip / damage
// per surface without coordinate hacks.
enum class SurfaceType : uintptr_t {
    None = 0,
    Road = 1,
    Grass = 2,
    Gravel = 3,
    Wall = 4,
};

inline void* SurfaceTag(SurfaceType type) {
    return reinterpret_cast<void*>(static_cast<uintptr_t>(type));
}

inline SurfaceType SurfaceFromTag(const void* tag) {
    return static_cast<SurfaceType>(reinterpret_cast<uintptr_t>(tag));
}

struct BodyDesc {
    Transform transform;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float mass = 0.f;          // 0 => static
    float friction = 0.4f;
    float restitution = 0.1f;
    float linearDamping = 0.f;
    float angularDamping = 0.05f;
    bool allowSleep = false;   // vehicles must never sleep
    void* userData = nullptr;
};

struct RayHit {
    bool hit = false;
    BodyHandle body = kInvalidBody;
    Vec3 point;
    Vec3 normal;
    float fraction = 0.f;      // 0..1 along the ray
    void* userData = nullptr;
    // Coulomb mu of the hit surface (from the hit body/shape). 0 when miss.
    float friction = 0.f;
};

// Contact reported for damage mapping. `localPointA/B` are contact points in
// each body's local space so the game layer can map them to car parts.
// `impactSpeed` is the CLOSING speed along worldNormal (relative velocity at
// the contact point projected on the normal) — resting/scraping contacts
// report ~0 even at 250 km/h, real crashes report their approach speed.
struct ContactEvent {
    BodyHandle bodyA = kInvalidBody;
    BodyHandle bodyB = kInvalidBody;
    Vec3 worldPoint;
    Vec3 worldNormal;          // from A toward B
    float impactSpeed = 0.f;
    void* userDataA = nullptr;
    void* userDataB = nullptr;
};

// Generic Coulomb mix — same geometric-mean rule as q3/rpc/nudge contact
// solvers. Used as the compatibility path when a backend lacks a native mix.
inline float MixFriction(float a, float b) {
    const float ua = a > 0.f ? a : 0.f;
    const float ub = b > 0.f ? b : 0.f;
    return std::sqrt(ua * ub);
}

// Which contact-friction / raycast extras the backend implements natively.
// Compat (non-native) paths still work but may only affect queries, not the
// contact solver (e.g. nudge has no runtime body-friction setter).
enum class PhysicsCapability : uint32_t {
    None = 0,
    NativeSetFriction = 1u << 0,       // SetFriction feeds the solver
    NativeRaycastFriction = 1u << 1,   // Raycast.friction from native shapes
    NativeContactFriction = 1u << 2,   // GetContactFriction uses native mix
};

inline PhysicsCapability operator|(PhysicsCapability a, PhysicsCapability b) {
    return static_cast<PhysicsCapability>(static_cast<uint32_t>(a) |
                                          static_cast<uint32_t>(b));
}
inline bool HasCapability(PhysicsCapability set, PhysicsCapability flag) {
    return (static_cast<uint32_t>(set) & static_cast<uint32_t>(flag)) != 0;
}

// Unified physics/collision interface. The vehicle model drives the world
// through this interface only, so qu3e can be swapped for Jolt (CCD, convex
// hulls) without touching gameplay code.
//
// Error contract (rush::Result = exstl::Result, no exceptions):
//  - Fallible body ops return Result; invalid handles are errors on RPC
//    (EX_FAIL). Qu3e/Nudge adapters always succeed — invalid handles yield
//    default/sentinel values wrapped as success (compat layer, no failure
//    branch). Raycast miss is not an error (RayHit.hit == false).
//  - Hot-path callers should use value_or / discard Result<void>; creation
//    paths may inspect the error. L1 physics cores stay assert/bool only —
//    Result lives in this L2 interface, not inside qu3e/nudge/rpc*.
class IPhysicsWorld {
public:
    virtual ~IPhysicsWorld() = default;

    virtual void SetGravity(const Vec3& gravity) = 0;
    virtual void Step(float dt) = 0;

    [[nodiscard]] virtual rush::Result<BodyHandle> CreateBody(const BodyDesc& desc) = 0;
    [[nodiscard]] virtual rush::Result<void> DestroyBody(BodyHandle body) = 0;

    [[nodiscard]] virtual rush::Result<Transform> GetTransform(BodyHandle body) const = 0;
    [[nodiscard]] virtual rush::Result<void> SetTransform(BodyHandle body,
                                                          const Transform& transform) = 0;

    [[nodiscard]] virtual rush::Result<Vec3> GetLinearVelocity(BodyHandle body) const = 0;
    [[nodiscard]] virtual rush::Result<Vec3> GetAngularVelocity(BodyHandle body) const = 0;
    [[nodiscard]] virtual rush::Result<void> SetLinearVelocity(BodyHandle body,
                                                               const Vec3& velocity) = 0;
    [[nodiscard]] virtual rush::Result<void> SetAngularVelocity(BodyHandle body,
                                                                const Vec3& velocity) = 0;
    [[nodiscard]] virtual rush::Result<Vec3> GetVelocityAtPoint(BodyHandle body,
                                                                const Vec3& worldPoint) const = 0;

    [[nodiscard]] virtual rush::Result<void> ApplyForce(BodyHandle body, const Vec3& force,
                                                        const Vec3& worldPoint) = 0;
    [[nodiscard]] virtual rush::Result<void> ApplyImpulse(BodyHandle body, const Vec3& impulse,
                                                          const Vec3& worldPoint) = 0;
    [[nodiscard]] virtual rush::Result<void> ApplyTorque(BodyHandle body, const Vec3& torque) = 0;

    // Raycast against all bodies. `ignore` skips one body (usually the caster).
    // On hit, friction is the surface Coulomb mu (native when supported).
    // Miss is not an error — returns RayHit{hit=false}.
    virtual RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                           BodyHandle ignore = kInvalidBody) const = 0;

    // Contacts from the most recent Step(). Used for damage part mapping.
    virtual std::vector<ContactEvent> DrainContacts() = 0;

    // --- Contact friction / extended raycast ------------------------------
    // RPC forwards these straight to rush-physics-core. Qu3e/Nudge forward
    // where the stock engine exposes the data; otherwise the adapter uses the
    // generic path (MixFriction / BodyDesc cache). See Capabilities().

    [[nodiscard]] virtual PhysicsCapability Capabilities() const = 0;

    // Coulomb mu stored on the body (first shape / body default).
    [[nodiscard]] virtual rush::Result<float> GetFriction(BodyHandle body) const = 0;

    // Runtime mu update. Native backends write through to the solver shapes;
    // compat backends may only update query-visible state (see Capabilities).
    [[nodiscard]] virtual rush::Result<void> SetFriction(BodyHandle body, float friction) = 0;

    // Effective contact mu for a pair (native mix or MixFriction fallback).
    [[nodiscard]] virtual rush::Result<float> GetContactFriction(BodyHandle a,
                                                                 BodyHandle b) const = 0;
};




// qu3e adapter implementing IPhysicsWorld. Chassis + track boundaries are
// boxes; suspension uses Raycast. Contact points are captured for damage
// part mapping. Swappable for a Jolt adapter later.
class Qu3eWorld final : public IPhysicsWorld {
public:
    // stepDt: q3Scene advances this fixed dt on every Step() — pass the game's
    // physics rate ([physics] hz) so the setting is real, not cosmetic.
    explicit Qu3eWorld(float stepDt = 1.f / 240.f);
    ~Qu3eWorld() override;

    void SetGravity(const Vec3& gravity) override;
    void Step(float dt) override;

    rush::Result<BodyHandle> CreateBody(const BodyDesc& desc) override;
    rush::Result<void> DestroyBody(BodyHandle body) override;

    rush::Result<Transform> GetTransform(BodyHandle body) const override;
    rush::Result<void> SetTransform(BodyHandle body, const Transform& transform) override;

    rush::Result<Vec3> GetLinearVelocity(BodyHandle body) const override;
    rush::Result<Vec3> GetAngularVelocity(BodyHandle body) const override;
    rush::Result<void> SetLinearVelocity(BodyHandle body, const Vec3& velocity) override;
    rush::Result<void> SetAngularVelocity(BodyHandle body, const Vec3& velocity) override;
    rush::Result<Vec3> GetVelocityAtPoint(BodyHandle body, const Vec3& worldPoint) const override;

    rush::Result<void> ApplyForce(BodyHandle body, const Vec3& force,
                                  const Vec3& worldPoint) override;
    rush::Result<void> ApplyImpulse(BodyHandle body, const Vec3& impulse,
                                    const Vec3& worldPoint) override;
    rush::Result<void> ApplyTorque(BodyHandle body, const Vec3& torque) override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore) const override;

    std::vector<ContactEvent> DrainContacts() override;

    PhysicsCapability Capabilities() const override;
    rush::Result<float> GetFriction(BodyHandle body) const override;
    rush::Result<void> SetFriction(BodyHandle body, float friction) override;
    rush::Result<float> GetContactFriction(BodyHandle a, BodyHandle b) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};




// nudge implementation of IPhysicsWorld. Boxes + raycast + contact summaries
// cover the vehicle/track needs; surface tags ride nudge's material palette
// (Material.user_data), so RayHit / ContactEvent report them like qu3e's
// per-body userData. Force/torque accumulate into velocity deltas because
// nudge exposes no force API; rotation sets recreate the body (no setter).
// Friction: create-time BodyParams only — SetFriction is compat (queries).
class NudgeWorld final : public IPhysicsWorld {
public:
    NudgeWorld();
    ~NudgeWorld() override;

    void SetGravity(const Vec3& gravity) override;
    void Step(float dt) override;

    rush::Result<BodyHandle> CreateBody(const BodyDesc& desc) override;
    rush::Result<void> DestroyBody(BodyHandle body) override;

    rush::Result<Transform> GetTransform(BodyHandle body) const override;
    rush::Result<void> SetTransform(BodyHandle body, const Transform& transform) override;

    rush::Result<Vec3> GetLinearVelocity(BodyHandle body) const override;
    rush::Result<Vec3> GetAngularVelocity(BodyHandle body) const override;
    rush::Result<void> SetLinearVelocity(BodyHandle body, const Vec3& velocity) override;
    rush::Result<void> SetAngularVelocity(BodyHandle body, const Vec3& velocity) override;
    rush::Result<Vec3> GetVelocityAtPoint(BodyHandle body, const Vec3& worldPoint) const override;

    rush::Result<void> ApplyForce(BodyHandle body, const Vec3& force,
                                  const Vec3& worldPoint) override;
    rush::Result<void> ApplyImpulse(BodyHandle body, const Vec3& impulse,
                                    const Vec3& worldPoint) override;
    rush::Result<void> ApplyTorque(BodyHandle body, const Vec3& torque) override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore = kInvalidBody) const override;

    std::vector<ContactEvent> DrainContacts() override;

    PhysicsCapability Capabilities() const override;
    rush::Result<float> GetFriction(BodyHandle body) const override;
    rush::Result<void> SetFriction(BodyHandle body, float friction) override;
    rush::Result<float> GetContactFriction(BodyHandle a, BodyHandle b) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};




// Rush Physics Core — first-party fork of qu3e (symbols rpc*), peer to
// Qu3eWorld / NudgeWorld under [physics] engine. Deep-customization
// **collision** backend only (island step, solver, queries). Vehicle
// dynamics stay on the shared Vehicle API in L2 and never move into
// rush-physics-core (see third_party/rush-physics-core/vehicle/README.md).
// Contact friction / raycast extras forward straight to rpc* native data.
// Unlike the qu3e/nudge compat adapters, RPC reports invalid handles and
// bad BodyDesc fields as rush::Result errors (EX_FAIL).
class RushPhysicsCore final : public IPhysicsWorld {
public:
    // stepDt: rpcScene advances this fixed dt on every Step() — pass the
    // game's physics rate ([physics] hz) so the setting is real.
    explicit RushPhysicsCore(float stepDt = 1.f / 240.f);
    ~RushPhysicsCore() override;

    void SetGravity(const Vec3& gravity) override;
    void Step(float dt) override;

    rush::Result<BodyHandle> CreateBody(const BodyDesc& desc) override;
    rush::Result<void> DestroyBody(BodyHandle body) override;

    rush::Result<Transform> GetTransform(BodyHandle body) const override;
    rush::Result<void> SetTransform(BodyHandle body, const Transform& transform) override;

    rush::Result<Vec3> GetLinearVelocity(BodyHandle body) const override;
    rush::Result<Vec3> GetAngularVelocity(BodyHandle body) const override;
    rush::Result<void> SetLinearVelocity(BodyHandle body, const Vec3& velocity) override;
    rush::Result<void> SetAngularVelocity(BodyHandle body, const Vec3& velocity) override;
    rush::Result<Vec3> GetVelocityAtPoint(BodyHandle body, const Vec3& worldPoint) const override;

    rush::Result<void> ApplyForce(BodyHandle body, const Vec3& force,
                                  const Vec3& worldPoint) override;
    rush::Result<void> ApplyImpulse(BodyHandle body, const Vec3& impulse,
                                    const Vec3& worldPoint) override;
    rush::Result<void> ApplyTorque(BodyHandle body, const Vec3& torque) override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore = kInvalidBody) const override;

    std::vector<ContactEvent> DrainContacts() override;

    PhysicsCapability Capabilities() const override;
    rush::Result<float> GetFriction(BodyHandle body) const override;
    rush::Result<void> SetFriction(BodyHandle body, float friction) override;
    rush::Result<float> GetContactFriction(BodyHandle a, BodyHandle b) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};




// Collision event bus: physics Step Publish()es contact events here; the
// game layer Drain()es on its own stage (never from inside Step).
rush::EventBus<ContactEvent>& CollisionBus();

// --- Vehicle dynamics (P2: moved from game layer) ---


// Per-wheel tire model:
//  - longitudinal: wheel-spin dynamics (omega) with an implicitly damped
//    contact force, so kappa is a real slip ratio (omega*r - v)/vRef and the
//    wheel can lock / spin. This is the hook TC and ABS modulate.
//  - lateral: load-sensitive Magic Formula on the transient (relaxation)
//    slip angle, reduced by longitudinal usage (friction ellipse) plus a
//    simplified camber-thrust term.
//  - wear accrues from sliding dissipation and lowers peak grip.
class TireModel {
public:
    struct Params {
        // Longitudinal: force = clamp(kStiff * peak * kappa, +/- peak), with
        // the implicit contact damper below. Peak near kappa ~ 1/kStiff.
        float kStiff = 11.f;

        // Lateral Magic Formula: D0*Fz0*(Fz/Fz0)^n *
        //   sin(C*atan(By*s - E*(By*s - atan(By*s)))),  s = tan(alpha)
        float C = 1.35f;
        float By = 11.5f;
        float E = 0.97f;
        float D0 = 1.60f;       // friction coefficient at reference load
        float Fz0 = 1840.f;     // reference vertical load (static corner) [N]
        float loadExp = 0.85f;  // n<1 => mu falls as load grows

        // Wheel spin dynamics: Iw * domega = Tdrive - Tbrake - Fx * r.
        float wheelInertia = 1.3f;   // kg m^2
        float wheelRadius = 0.33f;   // m

        // Lateral transient slip: first-order lag with this length [m].
        float relaxLength = 0.40f;

        // Simplified camber: thrust = camberCoef * Fz * lean (rad).
        float camberCoef = 3.0f;

        float wearRate = 2.5e-6f;
        float wearGripLoss = 0.35f;
        float minGrip = 0.55f;

        // F1-style split calibrated to real racing-slick behaviour: the
        // lateral curve peaks at 6.5-7.5 deg slip (By at C=1.35), the front
        // is the steering tire (slower build-up), the rear is stiffer and
        // more load-sensitive so heavy throttle / load transfer walks the
        // car into gentle oversteer.
        static Params Front() {
            Params p;
            p.By = 18.f;
            p.D0 = 1.58f;
            p.loadExp = 0.87f;
            p.wheelInertia = 1.2f;
            p.relaxLength = 0.35f;
            return p;
        }
        static Params Rear() {
            Params p;
            p.By = 20.f;
            p.D0 = 1.62f;
            p.loadExp = 0.83f;
            p.wheelInertia = 1.5f;
            p.relaxLength = 0.45f;
            return p;
        }
    };

    struct Output {
        Vec3 force;              // world-space tire force
        float slipRatio = 0.f;   // kappa = (omega*r - vLong)/vRef
        float slipAngleDeg = 0.f;
        float slipMagnitude = 0.f;
        float wearDelta = 0.f;
    };

    // Separate default ctor: clang rejects `= {}` as a default argument here
    // (default member initializers of Params are not yet available at that
    // point in the enclosing class).
    TireModel() = default;
    explicit TireModel(const Params& params);

    // contactVelocity: velocity of the contact patch (world).
    // forward/right: unit wheel frame axes in world (right = +X chassis axis,
    // i.e. the geometric LEFT side — see Vehicle.cpp frame notes).
    // driveForce/brakeForce: contact-plane forces [N] for this wheel.
    // camberLean: wheel-top lean toward the geometric right [rad].
    Output Compute(float verticalLoad, const Vec3& contactVelocity, const Vec3& forward,
                   const Vec3& right, float driveForce, float brakeForce, float dt,
                   float camberLean = 0.f);

    float Wear() const { return wear_; }
    void SetWear(float wear) { wear_ = Clampf(wear, 0.f, 1.f); }
    float GripMultiplier() const;
    float Omega() const { return omega_; }
    void Reset();

private:
    Params params_;
    float wear_ = 0.f;
    float omega_ = 0.f;      // wheel spin [rad/s]
    float alpha_ = 0.f;      // relaxed lateral slip angle [rad]
};




// Aerodynamic parts that can be damaged. Damage lowers part health in [0,1].
// How that health maps onto drag/downforce is a game rule — the game writes
// the resulting areas into VehicleInput; the engine only applies forces.
enum class CarPart : int {
    FrontWing = 0,
    RearWing = 1,
    Sidepods = 2,
    Floor = 3,
    Count = 4,
};

enum class WingMode : int {
    ZMode = 0,   // 2026 high-downforce (corners)
    XMode = 1,   // 2026 low-drag (straights)
};

// Driver aids, graded like the F1 games: Off / Sport / Full.
enum class AssistLevel : int {
    Off = 0,
    Sport = 1,
    Full = 2,
};

// Complete parameter set for one car. Defaults approximate a 2026-spec F1
// car at simcade level (Real Racing 3 / Monoposto feel).
struct VehicleParams {
    // Chassis
    float mass = 750.f;               // kg
    float chassisHalfX = 0.72f;       // half width [m]
    float chassisHalfY = 0.22f;       // half height [m]
    float chassisHalfZ = 1.85f;       // half length [m]
    float wheelbase = 3.60f;
    float trackWidth = 1.58f;
    float wheelRadius = 0.33f;

    // Suspension (per corner)
    float springK = 38000.f;          // N/m
    float damperC = 2100.f;           // Ns/m
    float suspensionRestLength = 0.16f;
    float suspensionMaxTravel = 0.22f;

    // Tires (front = steering tire, rear = drive tire; see Params::Front/Rear)
    TireModel::Params frontTire = TireModel::Params::Front();
    TireModel::Params rearTire = TireModel::Params::Rear();

    // Powertrain (rear-wheel drive)
    float maxTorque = 420.f;          // Nm at the crank
    float idleRpm = 3800.f;
    float peakRpm = 10500.f;
    float redlineRpm = 12500.f;
    float finalDrive = 3.0f;
    std::array<float, 8> gearRatios = {3.5f, 2.95f, 2.55f, 2.25f, 2.0f, 1.8f, 1.62f, 1.48f};
    float shiftUpRpm = 11800.f;
    float shiftDownRpm = 7200.f;
    float shiftTime = 0.06f;
    float engineBrakeTorque = 90.f;
    float brakeForceMax = 24000.f;    // N total

    // Aero  (force = 0.5 * rho * coeff * v^2)
    float airDensity = 1.225f;
    float dragArea = 1.02f;           // Cd*A
    float downforceArea = 5.7f;       // Cl*A ≈ 2.3x car weight at 250 km/h
    float frontDownforceShare = 0.44f;
    float centerOfPressureOffset = 0.1f;  // +Z moves aero balance rearward
    float aeroPitchBalance = 0.35f;   // front-share shift per rad of nose-up pitch

    // Assists (graded: Off / Sport / Full). Defaults match the playability
    // target: steering aid fully on, the rest at Sport.
    AssistLevel absLevel = AssistLevel::Sport;
    AssistLevel tcLevel = AssistLevel::Sport;
    AssistLevel escLevel = AssistLevel::Sport;
    AssistLevel steerLevel = AssistLevel::Full;

    // No-damage mode: collisions still bounce, but parts never lose health.
    bool damageEnabled = true;

    // Season rules ([season] in config): classic (DRS) and 2026 (active
    // aero + override) never mix. Game sets these from the season preset /
    // per-feature toggles at spawn; the vehicle gates activation itself so
    // a stale input can't leak a disabled system. Aero *effects* of DRS /
    // wing mode are game-computed (VehicleInput aero areas), not engine
    // formulas — only activation flags live here.
    bool drsEnabled = true;          // classic: DRS on straights
    bool activeAeroEnabled = false;  // 2026: X/Z wing modes (V)
    bool overtakeEnabled = false;    // 2026: manual override deploy (SHIFT)

    // ERS / overtake
    float ersCapacity = 4.0e6f;       // J
    float ersDeployPower = 350000.f;  // W
    float ersRegenPower = 250000.f;   // W under braking
    float ersBaseCharge = 0.55f;

    float overtakeMultiplier = 1.28f;
};

// Live state of one wheel.
struct WheelState {
    bool grounded = false;
    float suspensionLength = 0.f;
    float compression = 0.f;
    float load = 0.f;
    Vec3 contactPoint;
    Vec3 contactNormal;
    Vec3 contactVelocity;
    Vec3 force;
    float slipRatio = 0.f;
    float slipAngleDeg = 0.f;
    float wear = 0.f;
    float temperature = 70.f;
};

// Full per-frame vehicle state exposed to game/HUD/network.
struct VehicleState {
    Transform transform;
    Vec3 velocity;
    Vec3 angularVelocity;
    float speed = 0.f;              // m/s
    float speedKmh = 0.f;
    float forwardSpeed = 0.f;       // signed, + = moving forward
    float rpm = 0.f;
    int gear = 1;                   // 1..N, 0 = reverse
    float throttle = 0.f;
    float brake = 0.f;
    float steerAngleDeg = 0.f;
    std::array<WheelState, 4> wheels;
    std::array<float, 4> partHealth;  // CarPart indexed, 1 = intact
    float ersCharge = 0.f;            // 0..1
    bool ersDeploying = false;
    bool drsActive = false;
    bool drsAvailable = false;
    WingMode wingMode = WingMode::ZMode;
    float frontDownforce = 0.f;
    float rearDownforce = 0.f;
    float dragForce = 0.f;
    float damageTotal = 0.f;
    float wheelSpin = 0.f;  // visual wheel rotation [rad]
};

struct VehicleInput {
    float throttle = 0.f;
    float brake = 0.f;
    float steer = 0.f;
    bool handbrake = false;
    bool reverse = false;
    bool drsRequest = false;  // driver wants DRS open (latched by the game)
    bool drsZone = false;     // car is inside a map-marked DRS activation zone
    bool wingToggle = false;
    bool overtake = false;

    // Effective aero areas [m^2] after game rules (damage, DRS, wing mode,
    // pitch balance). Written by the game each substep; the engine only
    // turns them into forces (q * A) — it never derives them from DRS /
    // X-Mode / part health. Negative = unset: fall back to VehicleParams
    // base areas + pitch split (unit tests that skip the game layer).
    float dragArea = -1.f;
    float frontDownforceArea = -1.f;
    float rearDownforceArea = -1.f;
};

// Chassis rigid body + 4 raycast-suspension wheels + combined-slip tires.
// All physics goes through IPhysicsWorld so the collision backend is
// replaceable (qu3e today, Jolt later).
class Vehicle {
public:
    Vehicle() = default;

    void Spawn(IPhysicsWorld& world, const Transform& pose, const VehicleParams& params,
               void* userData = nullptr);
    void Despawn();

    // One fixed physics substep. Applies suspension/tire/aero forces to the
    // chassis body; the caller then steps the physics world.
    void Update(const VehicleInput& input, float dt);

    void ProcessContacts(const std::vector<ContactEvent>& contacts, float dt);

    void Reset(const Transform& pose);

    const VehicleState& State() const { return state_; }
    const VehicleParams& Params() const { return params_; }
    BodyHandle Chassis() const { return chassis_; }

    // Speed-sensitive full-lock road-wheel angle [deg]. Full lock sits at
    // 1.2x the kinematic grip angle delta = L * a_max(v) / v^2 — the extra
    // 20% covers the tire slip-angle headroom (L/R + understeer gradient) so
    // the limit is reachable; the oversteer-gated assists catch anything past
    // it. Derived from the tire / aero calibration, shared with the AI.
    float SteerEnvelopeDeg(float speed) const;
    float PartHealth(CarPart part) const {
        return state_.partHealth[static_cast<int>(part)];
    }

    void Render(IRenderer& renderer, const Color& tint) const;
    // Snapshot-friendly draw: no IPhysicsWorld access — used by the async
    // render stage from a published VehicleState + chassis transform.
    static void RenderPose(IRenderer& renderer, const Transform& chassis,
                           const VehicleState& state, const VehicleParams& params,
                           const Color& tint);

private:
    void UpdatePowertrain(const VehicleInput& input, float dt);
    void UpdateWheels(float dt);
    void UpdateAssists(VehicleInput& input);
    void UpdateDamageFromContact(const ContactEvent& event);

    IPhysicsWorld* world_ = nullptr;
    BodyHandle chassis_ = kInvalidBody;
    VehicleParams params_;
    VehicleState state_;

    std::array<TireModel, 4> tires_;
    std::array<Vec3, 4> hardpoints_;  // suspension attach points in local space

    // Powertrain
    float engineRpm_ = 0.f;
    int gear_ = 1;
    float shiftTimer_ = 0.f;
    float driveTorque_ = 0.f;

    // Input smoothing
    float steerAngle_ = 0.f;
    float throttleSmooth_ = 0.f;
    float brakeSmooth_ = 0.f;

    // ESC intervention state from UpdateAssists, consumed by UpdatePowertrain
    // (rear brake reserve while cornering) and UpdateAssists itself.
    float escStrength_ = 0.f;

    // Per-wheel drive/brake force split (N), filled by UpdatePowertrain.
    std::array<float, 4> drivePerWheel_{};
    std::array<float, 4> brakePerWheel_{};

    // Mode state
    bool wingTogglePrev_ = false;

    // Visual-only wheel rotation (radians), accumulated from road speed.
    float wheelSpinAngle_ = 0.f;
};


}  // namespace apex
