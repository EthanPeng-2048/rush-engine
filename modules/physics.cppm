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

// One-shot chassis state fetch: the vehicle hot path reads transform +
// velocities every substep — three separate getters would cost three handle
// lookups per substep. Invalid handles yield a default snapshot (compat).
struct ChassisSnapshot {
    Transform transform;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
};

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

struct ShapeDesc {
    Transform localTransform;      // pose relative to the body
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    int partTag = -1;              // part label (CarPart value), -1 = none.
                                   // Type-safe: CarPart 0 must not collide
                                   // with "no tag" (void* 0 == nullptr).
    void* userData = nullptr;      // arbitrary payload (unused by vehicles)
    float friction = 0.f;          // 0 = inherit body friction
};

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
    // Multi-shape (composite rigid body): empty = legacy single box
    // (halfExtents). Non-empty backends place every shape on the body and
    // derive mass/inertia from density * sum of volumes.
    std::vector<ShapeDesc> shapes;
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
    // Shape tags of the boxes involved (partA belongs to bodyA's box,
    // partB to bodyB's). -1 = unknown (legacy single-box / nudge backend).
    int partA = -1, partB = -1;
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

    // Batch fetch for the per-substep vehicle hot path (transform + linear +
    // angular velocity in one backend round-trip instead of three lookups).
    [[nodiscard]] virtual rush::Result<ChassisSnapshot> FetchChassis(BodyHandle body) const = 0;

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

    // Batch raycast: `count` rays sharing one direction and maxDistance (the
    // suspension hot path — 4 wheel hardpoints cast straight down). RPC
    // traverses the broadphase once for all rays; qu3e/nudge loop the single-
    // ray query. Same results as `count` Raycast calls; each `out[j]` holds
    // that ray's closest hit (miss keeps hit == false). count <= 32.
    virtual void RaycastBatch(const Vec3* origins, const Vec3& direction, float maxDistance,
                              int count, BodyHandle ignore, RayHit* out) const = 0;

    // Contacts from the most recent Step(). Used for damage part mapping.
    virtual std::vector<ContactEvent> DrainContacts() = 0;

    // Optional parallel island solving (RPC backend). Bit-identical numerics
    // to serial — only execution order changes. Default off: deterministic
    // paths (drive-test) never enable it. Other backends ignore the call.
    virtual void SetParallelIslands(bool enabled) { (void)enabled; }

    // Per-stage cost, for benchmarks and tuning. Reported as MEANS over every
    // Step() since construction (a quiet final substep would otherwise fake a
    // zero report); peaks are reported separately. Backends that do not
    // instrument themselves leave the default (steps == 0), which callers must
    // read as "not available", never as "free".
    struct StageStats {
        double broadphaseMs = 0.0;   // proxy sync + pair generation
        double narrowphaseMs = 0.0;  // contact manifold generation
        double islandBuildMs = 0.0;  // island graph + solver setup
        double solveMs = 0.0;        // constraint iterations
        double proxyMs = 0.0;        // per-shape AABB + index updates
        double syncMs = 0.0;         // whole SyncAndFind (proxy + pairs + clears)
        double pairMs = 0.0;         // pair generation (queries + sort + contacts)
        double forceMs = 0.0;        // clearing per-body force accumulators
        double stepMs = 0.0;         // whole backend Step() (closes the accounting)
        double wrapMs = 0.0;         // adapter work around Step(): force push + events
        double pairQueryMs = 0.0;    // of pairMs: broadphase tree queries
        double sortMs = 0.0;         // of pairMs: sort + dedupe + contact creation
        int movedProxies = 0;        // peak shape proxies re-queried per step
        int pairsProduced = 0;       // peak candidate pairs per step
        int pairsUnique = 0;         // peak unique pairs per step
        int contacts = 0;            // peak live contacts
        int contactPoints = 0;       // peak contact points
        int islands = 0;             // peak island count
        int gridCells = 0;           // static ray-grid cells (0 = no grid)
        long steps = 0;              // samples behind the means (0 = no data)
    };
    virtual void ReadStageStats(StageStats& out) const { (void)out; }
    // Drop accumulated stage means (bench warm-up → measured window).
    virtual void ResetStageStats() {}
    // Toggle the per-stage timers above (off by default: they cost clock reads).
    virtual void SetDiagnostics(bool on) { (void)on; }

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
    rush::Result<ChassisSnapshot> FetchChassis(BodyHandle body) const override;

    rush::Result<void> ApplyForce(BodyHandle body, const Vec3& force,
                                  const Vec3& worldPoint) override;
    rush::Result<void> ApplyImpulse(BodyHandle body, const Vec3& impulse,
                                    const Vec3& worldPoint) override;
    rush::Result<void> ApplyTorque(BodyHandle body, const Vec3& torque) override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore) const override;

    void RaycastBatch(const Vec3* origins, const Vec3& direction, float maxDistance, int count,
                      BodyHandle ignore, RayHit* out) const override;

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
    rush::Result<ChassisSnapshot> FetchChassis(BodyHandle body) const override;

    rush::Result<void> ApplyForce(BodyHandle body, const Vec3& force,
                                  const Vec3& worldPoint) override;
    rush::Result<void> ApplyImpulse(BodyHandle body, const Vec3& impulse,
                                    const Vec3& worldPoint) override;
    rush::Result<void> ApplyTorque(BodyHandle body, const Vec3& torque) override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore = kInvalidBody) const override;

    void RaycastBatch(const Vec3* origins, const Vec3& direction, float maxDistance, int count,
                      BodyHandle ignore, RayHit* out) const override;

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
    rush::Result<ChassisSnapshot> FetchChassis(BodyHandle body) const override;

    rush::Result<void> ApplyForce(BodyHandle body, const Vec3& force,
                                  const Vec3& worldPoint) override;
    rush::Result<void> ApplyImpulse(BodyHandle body, const Vec3& impulse,
                                    const Vec3& worldPoint) override;
    rush::Result<void> ApplyTorque(BodyHandle body, const Vec3& torque) override;

    void SetParallelIslands(bool enabled) override;
    void ReadStageStats(StageStats& out) const override;
    void SetDiagnostics(bool on) override;
    void ResetStageStats() override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore = kInvalidBody) const override;

    void RaycastBatch(const Vec3* origins, const Vec3& direction, float maxDistance, int count,
                      BodyHandle ignore, RayHit* out) const override;

    std::vector<ContactEvent> DrainContacts() override;

    PhysicsCapability Capabilities() const override;
    rush::Result<float> GetFriction(BodyHandle body) const override;
    rush::Result<void> SetFriction(BodyHandle body, float friction) override;
    rush::Result<float> GetContactFriction(BodyHandle a, BodyHandle b) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    // Stage-timing accumulators. ReadStageStats reports means over every step
    // since construction, so a quiet final substep cannot fake a zero report.
    double stageBroadMs_ = 0.0;
    double stageProxyMs_ = 0.0;
    double stagePairMs_ = 0.0;
    double stageNarrowMs_ = 0.0;
    double stageBuildMs_ = 0.0;
    double stageSolveMs_ = 0.0;
    double stageSyncMs_ = 0.0;
    double stageForceMs_ = 0.0;
    double stageStepMs_ = 0.0;
    double stageWrapMs_ = 0.0;
    double stagePairQueryMs_ = 0.0;
    double stageSortMs_ = 0.0;
    int stageMovedPeak_ = 0;
    int stagePairsProducedPeak_ = 0;
    int stagePairsUniquePeak_ = 0;
    bool diag_ = false;
    long stageSteps_ = 0;
    int stageContactsPeak_ = 0;
    int stagePointsPeak_ = 0;
    int stageIslandsPeak_ = 0;
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
        float alignMoment = 0.f; // self-aligning yaw moment (body-up axis):
                                 // -fyLat * casterTrail, positive restores a
                                 // right-hand turn (negative yaw) to center.
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
    // slaved: wheel spin is externally commanded (driven axle slaved to the
    // crank through the gear); the tire then only resolves the slip force —
    // no free spin integration, drive/brake torque no longer spins the wheel.
    // casterTrail: suspension caster trail [m]; the lateral force then emits
    // a self-aligning yaw moment (0 = no align moment, e.g. rear wheels).
    Output Compute(float verticalLoad, const Vec3& contactVelocity, const Vec3& forward,
                   const Vec3& right, float driveForce, float brakeForce, float dt,
                   float camberLean = 0.f, bool slaved = false,
                   float omegaCommanded = 0.f, float casterTrail = 0.f);

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

// ============================================================================
// Component parameter sets. Each car component owns its tuning knobs so the
// game can swap parts (改车) or replace whole systems (engine/gearbox) without
// touching the others.

// Engine: torque curve, crankshaft dynamics (I_c * dw = T_gas - T_fric -
// T_load), simplified combustion (firing pulses / rev limiter / misfire) and
// the 2026 hybrid ERS store. rpm is a force-balance result, not a table.
struct EngineParams {
    float maxTorque = 420.f;          // Nm at the crank (gas torque, WOT, peak)
    float idleRpm = 3800.f;
    float peakRpm = 10500.f;
    float redlineRpm = 12500.f;

    // Crankshaft dynamics
    float crankInertia = 0.45f;       // kg m^2
    int cylinders = 6;                // firing rate = rpm/60 * cylinders/2
    float intakeLag = 0.025f;         // s — throttle -> airflow (first order)
    float torquePulse = 0.05f;        // per-firing torque ripple (frac of maxTorque)
    float misfireRate = 0.02f;        // per firing event
    uint32_t misfireSeed = 0x5EED2026u;  // fixed seed: replay-safe randomness

    // Rev limiter (hysteretic fuel cut)
    float fuelCutRpm = 12500.f;
    float fuelCutResumeRpm = 12150.f;

    // Engine braking = crank friction + throttle pumping (replaces the old
    // wheel-side engineBrakeTorque).
    float frictionC0 = 12.f;          // Nm base
    float frictionC1 = 0.004f;        // Nm per rad/s
    float pumpingTorque = 25.f;       // Nm at fully closed throttle
    float idleHoldTorque = 320.f;     // idle governor cap [Nm]

    // ERS / overtake (2026 hybrid store)
    float ersCapacity = 4.0e6f;       // J
    float ersDeployPower = 350000.f;  // W
    float ersRegenPower = 250000.f;   // W under braking
    float ersBaseCharge = 0.55f;
    float overtakeMultiplier = 1.28f;
};

// Transmission: gear ratios / final drive / shift logic + drive & brake split.
struct TransmissionParams {
    float finalDrive = 3.0f;
    std::array<float, 8> gearRatios = {3.5f, 2.95f, 2.55f, 2.25f, 2.0f, 1.8f, 1.62f, 1.48f};
    float shiftUpRpm = 11800.f;
    float shiftDownRpm = 7200.f;
    float shiftTime = 0.06f;
    float brakeForceMax = 24000.f;    // N total
};

// Clutch-type LSD on the driven axle. The diff case is rigidly coupled to the
// crank (omega_case = crank/ratio, average wheel speed locked); the only free
// degree is the split delta = (wL - wR)/2. Bias torque T_b clamps to
// +/- lockFactor*|T_axle|, so 1 = spool, 0 = open. Acceleration and decel
// (engine braking) run separate parameter sets: F1 LSDs lock hard on power,
// open up under braking so the tail does not step out.
struct DifferentialParams {
    float preloadAccel = 40.f;        // Nm base bias on power
    float rampAccel = 120.f;          // Nm per rad/s of (wL - wR)
    float lockFactorAccel = 0.65f;    // bias cap as fraction of |T_axle|
    float preloadDecel = 8.f;         // Nm base bias under engine braking
    float rampDecel = 30.f;           // Nm per rad/s
    float lockFactorDecel = 0.35f;
    float biasRateLimit = 4000.f;     // Nm/s slew limit on the bias torque
    float deltaDamping = 20.f;        // 1/s — split-state damper. The tire
                                      // force difference only restores the
                                      // split in the unsaturated regime;
                                      // under full spin-up (launch) both
                                      // tires saturate and the bias would
                                      // otherwise grow the split without
                                      // bound. Models internal diff friction.
    float deltaMax = 1.0f;            // rad/s — clutch slip limit of the LSD:
                                      // the commanded per-wheel spin offset
                                      // is clamped to +/- deltaMax, i.e. the
                                      // split never outruns the case command
                                      // (a locked case is delta == 0).
};

// Suspension: spring/damper per axle (front/rear independent), travel, camber,
// anti-roll bars and the equivalent geometry parameters (caster trail for
// self-aligning torque, static toe, per-axle camber, roll steer).
struct SuspensionParams {
    float springKf = 38000.f;         // front spring N/m
    float damperCf = 2100.f;          // front damper Ns/m
    float springKr = 38000.f;         // rear spring N/m
    float damperCr = 2100.f;          // rear damper Ns/m
    float suspensionRestLength = 0.16f;
    float suspensionMaxTravel = 0.22f;

    // Anti-roll bars: equal-and-opposite load correction from the axle's
    // compression difference (0 = no bar). Front/rear ratio tunes the roll
    // stiffness split = cornering balance.
    float kArbF = 9000.f;             // N/m equivalent
    float kArbR = 12000.f;

    // Caster trail: front self-aligning torque = Fy * trail, applied as yaw
    // moment about the body up axis (rear wheels run trail = 0).
    float casterTrailF = 0.035f;      // m

    // Static toe (rad): front on top of steer, rear direct.
    float toeF = 0.0015f;             // ~0.09 deg toe-in
    float toeR = -0.0005f;

    // Camber per axle (static negative) and roll claw-back ratio.
    float camberStaticF = 0.030f;
    float camberStaticR = 0.040f;
    float camberGainF = 0.60f;
    float camberGainR = 0.70f;

    // Roll steer (rad/rad): steer += rollSteerF * frontRoll (default off).
    float rollSteerF = 0.f;
};

// Aero kit: front wing / rear wing / sidepods / floor. Owns part health and
// the damage→area / DRS / X-Z / pitch rules (was game-layer WriteVehicleAero).
// Also the directional-stability derivatives: body slip beta feeds an aero
// side force and yaw moment (the >80 km/h stability source in real F1).
struct AeroKitParams {
    float airDensity = 1.225f;
    float dragArea = 1.02f;           // Cd*A
    float downforceArea = 5.7f;       // Cl*A ≈ 2.3x car weight at 250 km/h
    float frontDownforceShare = 0.44f;
    float centerOfPressureOffset = 0.1f;  // +Z moves aero balance rearward
    float aeroPitchBalance = 0.35f;   // front-share shift per rad of nose-up pitch

    // Directional stability: side force = -q*sideArea*ClBeta*beta along the
    // body right axis (geometric left), yaw moment = -q*sideArea*Lref*
    // (CnBeta*beta + CnR*yawRate) about body up. q = 0.5*rho*v^2.
    float sideArea = 1.15f;           // m^2 lateral reference area
    float aeroClBeta = 2.6f;          // /rad side-force derivative
    float aeroCnBeta = 0.9f;          // /rad yaw-moment derivative
    float aeroCnR = 0.10f;            // 1/(rad/s) yaw damping derivative
    float aeroRefLength = 3.6f;       // m moment reference length (= wheelbase)

    // COP height above CG: the aero resultant acts at this height, so the
    // side force produces roll moment (balanced by the suspension roll
    // stiffness) and drag/downforce keep their pitch moments.
    float copHeight = 0.22f;          // m
};

// Driver assistance (TCS/ESC/ABS): modulates actuator requests, NOT input.
struct DriverAssistanceParams {
    AssistLevel absLevel = AssistLevel::Sport;
    AssistLevel tcLevel = AssistLevel::Sport;
    AssistLevel escLevel = AssistLevel::Sport;

    // TCS
    float tcThreshFull = 0.115f;
    float tcThreshSport = 0.20f;
    float tcCutGain = 6.1f;
    float tcBudgetReserveFull = 0.94f;   // traction budget for the driven axle
    float tcBudgetReserveSport = 1.10f;

    // ABS
    float absThreshFull = 0.115f;
    float absThreshSport = 0.18f;
    float absReleaseGain = 6.f;
    float absMaxRelease = 0.91f;

    // ESC
    float escStrengthFull = 1.f;
    float escStrengthSport = 0.58f;
    float escYawFull = 6.6f;
    float escYawSport = 4.3f;
    float escLateral = 1.9f;
    float escRearBrakeReserve = 0.39f;
    float escRearBrakeGain = 5.7f;
    float escOversteerYaw = 0.25f;
    float escOversteerBeta = 0.06f;
    float escBetaDeepFull = 0.14f;
    float escBetaDeepSport = 0.18f;
};

// Driver controls: keyboard smoothing + counter-steer/anti-spin. This is
// input processing — explicitly NOT part of the assistance component.
struct DriverControlsParams {
    float steerTauUnwind = 0.11f;          // faster release easing
    float steerTauWindBase = 0.10f;
    float steerTauWindPerSpeed = 0.0032f;  // wind-up slows with speed
    float steerTauWindMax = 0.34f;
    float throttleRate = 10.f;             // /s easing toward full
    float brakeRate = 14.f;
    float counterSteerMaxFull = 0.85f;     // of steer range
    float counterSteerMaxSport = 0.45f;
    float counterSteerBetaNorm = 0.15f;
    float counterSteerExcessNorm = 0.6f;
    float oversteerYaw = 0.2f;
    float oversteerBeta = 0.06f;
    float oversteerBetaDeep = 0.18f;
    float counterSteerMinSpeed = 6.f;
    AssistLevel steerLevel = AssistLevel::Full;
};

// Complete parameter set for one car. Defaults approximate a 2026-spec F1
// car at simcade level (Real Racing 3 / Monoposto feel).
struct PartBoxDesc {
    Vec3 position = {};          // local offset from the chassis origin
    Vec3 halfExtents = {};       // box half size (0 = no box)
};

struct VehicleParams {
    // Chassis
    float mass = 750.f;               // kg
    float wheelbase = 3.60f;
    float trackWidth = 1.58f;
    float wheelRadius = 0.33f;

    // Legacy single collision box (nudge backend / fallback; qu3e/rpc use
    // the multi-box layout below when partBoxes is populated).
    float chassisHalfX = 0.72f;       // half width [m]
    float chassisHalfY = 0.22f;       // half height [m]
    float chassisHalfZ = 1.85f;       // half length [m]

    // Composite collision layout (qu3e/rpc): one box per CarPart plus the
    // main chassis box. Sidepods position.x is the symmetric +/- offset.
    // The main box carries no part tag (damage maps it via the fallback).
    std::array<PartBoxDesc, 4> partBoxes{{
        {{0.f, 0.06f, 1.68f}, {0.72f, 0.05f, 0.18f}},    // FrontWing
        {{0.f, 0.62f, -1.66f}, {0.70f, 0.12f, 0.15f}},   // RearWing
        {{0.70f, 0.20f, 0.f}, {0.14f, 0.16f, 1.05f}},    // Sidepods (L/R)
        {{0.f, -0.02f, 0.5f}, {0.55f, 0.06f, 1.0f}},     // Floor
    }};
    PartBoxDesc chassisBox{{0.f, 0.15f, 0.f}, {0.58f, 0.18f, 1.35f}};

    // Tires (front = steering tire, rear = drive tire; see Params::Front/Rear)
    TireModel::Params frontTire = TireModel::Params::Front();
    TireModel::Params rearTire = TireModel::Params::Rear();

    // Components — replace / tune each independently (改车).
    EngineParams engine;
    TransmissionParams gearbox;
    DifferentialParams differential;
    SuspensionParams suspension;
    AeroKitParams aero;
    DriverAssistanceParams assists;
    DriverControlsParams controls;

    // No-damage mode: collisions still bounce, but parts never lose health.
    bool damageEnabled = true;

    // Season rules ([season] in config): classic (DRS) and 2026 (active
    // aero + override) never mix. Game sets these at spawn; the vehicle
    // gates activation itself so a stale input can't leak a disabled system.
    bool drsEnabled = true;          // classic: DRS on straights
    bool activeAeroEnabled = false;  // 2026: X/Z wing modes (V)
    bool overtakeEnabled = false;    // 2026: manual override deploy (SHIFT)
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
    float ersCharge = 0.f;            // 0..1
    bool ersDeploying = false;
    bool drsActive = false;
    bool drsAvailable = false;
    WingMode wingMode = WingMode::ZMode;
    float frontDownforce = 0.f;
    float rearDownforce = 0.f;
    float dragForce = 0.f;
    float wheelSpin = 0.f;  // visual wheel rotation [rad]
};

// Driver intent. Written raw by the game / AI; Vehicle::Update runs it through
// DriverControls (smoothing / counter-steer) before the components see it.
// Aero areas are NOT here — they live in AeroKit (part health / DRS / X-Z).
struct CarInput {
    float throttle = 0.f;
    float brake = 0.f;
    float steer = 0.f;
    bool handbrake = false;
    bool reverse = false;
    bool drsRequest = false;  // driver wants DRS open (latched by the game)
    bool drsZone = false;     // car is inside a map-marked DRS activation zone
    bool wingToggle = false;
    bool overtake = false;
};

// Read-only chassis frame handed to components each substep.
struct CarFrame {
    IPhysicsWorld* world = nullptr;   // components may ApplyForce / Raycast
    Transform chassis;
    Vec3 velocity, angularVelocity;
    Vec3 forward, right, up;   // +Z fwd, +X geometric-left, +Y up (vehicle frame)
    float speed = 0.f, forwardSpeed = 0.f;
    float dt = 0.f;
};

// Speed-sensitive full-lock road-wheel angle [deg]. Full lock sits at 1.2x the
// kinematic grip angle delta = L * a_max(v) / v^2 (see Vehicle::SteerEnvelope).
float SteerEnvelopeDeg(const VehicleParams& p, float speed);

// ============================================================================
// Car components. Each owns Params + State + Step; none touches L1 directly
// except where the job is geometric (Suspension raycast, AeroKit reads frame).

// --- Engine ---------------------------------------------------------------
// Crank dynamics against a driveline load (I_c * dw = T_gas - T_fric - T_load)
// with simplified combustion: intake lag, per-firing torque pulses, hysteretic
// rev-limiter fuel cut, seeded misfire, idle governor. The crank owns its rpm;
// the driven wheels are slaved to it through the current gear.
class Engine {
public:
    struct State {
        float rpm = 0.f;              // crank [rpm] (>= 0; floor prevents stall)
        float omega = 0.f;            // crank [rad/s]
        float firingPhase = 0.f;      // [0,1) within one firing interval (audio phase source)
        float intakeFill = 0.f;       // throttle lag state [0..1]
        bool fuelCut = false;
        float ersCharge = 0.55f;
        bool ersDeploying = false;
    };

    struct StepInput {
        float throttle = 0.f;         // smoothed + assistance-cut throttle [0..1]
        float loadTorque = 0.f;       // driveline load reflected to the crank [Nm]
        float ersPower = 0.f;         // hybrid deploy power [W] -> crank torque
        float inertia = 0.f;          // total effective inertia at the crank [kg m^2]
        float dt = 0.f;
    };

    struct StepOutput {
        float crankTorque = 0.f;      // net driveline torque (gas - friction + ers) [Nm]
        float rpm = 0.f;
        float firingPhase = 0.f;
        bool fuelCut = false;
        bool misfire = false;
    };

    void Init(const EngineParams& p) {
        params_ = p;
        st_ = State{};
        st_.ersCharge = p.ersBaseCharge;
        rng_ = p.misfireSeed;
    }
    // One crank step: intake lag -> fuel cut -> firing pulses/misfire -> idle
    // governor -> integrate omega against the load.
    StepOutput Step(const StepInput& in);
    // ERS: deploy on overtake / regen under braking. Returns deploy power [W]
    // (or 0) after updating charge. throttle/brake are the smoothed values.
    float UpdateErc(const CarInput& intent, float throttle, float brake, float speed,
                    bool overtakeEnabled, float dt);
    // Rigid-driveline ratio change (gear shift): omega *= factor.
    void Rescale(float factor);
    const State& StateRef() const { return st_; }
    void Reset();

private:
    float TorqueFactor() const;
    float FrictionTorque(float throttleEff) const;
    EngineParams params_;
    State st_;
    uint32_t rng_ = 0u;               // xorshift misfire rolls (seeded, deterministic)
};

// --- Transmission ----------------------------------------------------------
// Gear ratios / final drive / shift logic + axle-level drive, per-wheel brake.
class Transmission {
public:
    struct State { int gear = 1; float shiftTimer = 0.f; };  // gear 0 = reverse
    struct Output {
        float rearAxleDrive = 0.f;       // rear-axle total drive [N] (signed:
                                         // negative in reverse / engine braking)
        std::array<float, 4> brakePerWheel{};
    };

    void Init(const TransmissionParams& p) { params_ = p; st_ = State{}; }
    // Shift logic + drive/brake split. `crankTorque` is the engine's net
    // driveline torque (throttle-inclusive, engine braking included, ERS
    // already added at the crank); the Differential component then splits the
    // axle drive between the driven wheels. rearBrakeScale comes from
    // assistance (ESC tail-brake reserve).
    Output Compute(const CarInput& intent, float crankTorque, float rpm, float speed,
                   float forwardSpeed, float wheelRadius, float rearBrakeScale, float dt);
    const State& StateRef() const { return st_; }
    void Reset();

private:
    TransmissionParams params_;
    State st_;
};

// --- Differential (clutch-type LSD on the driven axle) ----------------------
// Case kinematics: (wL + wR)/2 = crank/ratio, enforced by the orchestrator
// commanding each driven wheel at caseOmega +/- split (slaved). The split
// state delta = (wL - wR)/2 feeds the bias law; its dynamics live in the
// orchestrator, which owns the tire reaction forces:
//   d(delta)/dt = (bias - (FxL - FxR)*r) / (2*Iw) - deltaDamping*delta
// Splits the axle drive, never the brakes (ABS stays per-wheel).
class Differential {
public:
    struct State {
        float delta = 0.f;   // (wL - wR)/2 [rad/s], fed back into the bias law
        float bias = 0.f;    // current bias torque [Nm] (HUD/debug readout)
    };
    struct Input {
        float axleDrive = 0.f;  // rear-axle total drive [N] (signed)
        float wheelRadius = 0.f;
        float dt = 0.f;
        float fxTorqueDiff = 0.f;  // (FxL - FxR) * r: tire reaction diff [Nm]
        float wheelInertia = 0.f;  // per driven wheel [kg m^2]
    };
    struct Output { float driveL = 0.f, driveR = 0.f; };  // wheel-end drive [N]

    void Init(const DifferentialParams& p) { params_ = p; st_ = State{}; }
    // Bias law (clamp to +/- lockFactor*|T_axle|, slew-limited) -> split,
    // then integrate the split state against tire reactions and damping.
    Output Step(const Input& in);
    const State& StateRef() const { return st_; }
    void Reset() { st_ = State{}; }

private:
    DifferentialParams params_;
    State st_;
};

// --- Suspension (front/rear) -----------------------------------------------
// 4 raycast springs/dampers → per-corner load + contact, anti-roll bars, and
// the equivalent geometry (toe, per-axle camber, roll steer). Front/rear
// params independent. Emits the simplified camber for the tire model.
class Suspension {
public:
    struct Corner {
        bool grounded = false;
        float compression = 0.f, load = 0.f, suspensionLength = 0.f;
        SurfaceType surface = SurfaceType::None;
        Vec3 contactPoint, contactNormal, contactVelocity;
        // Wheel frame axes in world (toe applied, steer NOT — the orchestrator
        // adds the ESC-corrected steer + roll steer on top).
        Vec3 wheelForward, wheelRight;
    };
    struct State { std::array<Corner, 4> corner; };

    void Init(const SuspensionParams& p) { params_ = p; st_ = State{}; }
    // Per-wheel raycast. hardpoints are chassis-local attach points; front
    // wheels get static toe (steer is applied by the orchestrator with the
    // assistance trim). Fills camber[4] (rad) for the tire model.
    void Step(const CarFrame& frame, BodyHandle chassis, float wheelRadius,
              float trackWidth, const std::array<Vec3, 4>& hardpoints,
              float camber[4], float dt);
    const State& StateRef() const { return st_; }
    void Reset();

private:
    SuspensionParams params_;
    State st_;
};

// --- Aero kit (front wing / rear wing / sidepods / floor) ------------------
// Owns part health and the damage→area / DRS / X-Z / pitch rules — this was
// the game-layer WriteVehicleAero.
class AeroKit {
public:
    struct State {
        float frontDownforce = 0.f, rearDownforce = 0.f, dragForce = 0.f;
        std::array<float, 4> partHealth{1.f, 1.f, 1.f, 1.f};  // CarPart indexed
    };
    struct Output {
        Vec3 cop;                      // aero resultant application point
                                       // (chassis space, raised by copHeight)
        float frontDF = 0.f, rearDF = 0.f, drag = 0.f;
        Vec3 sideForce;                // directional-stability side force (world)
        Vec3 yawMoment;                // directional-stability yaw moment (world)
    };

    void Init(const AeroKitParams& p) { params_ = p; st_ = State{}; }
    void ApplyDamage(CarPart part, float amount);
    // damage→area / DRS / X-Z / pitch rules → aero forces (was WriteVehicleAero).
    Output Compute(const CarFrame& frame, WingMode wingMode, bool drsActive);
    float DamageTotal() const;
    const State& StateRef() const { return st_; }
    void Reset();

private:
    AeroKitParams params_;
    State st_;
};

// --- Driver assistance (TCS / ESC / ABS) -----------------------------------
// Modulates actuator requests from car state — NOT input processing.
class DriverAssistance {
public:
    struct State { float escStrength = 0.f; };
    struct Output {
        float throttle = 0.f;        // post-TC cut
        float steer = 0.f;           // post-ESC trim (rad)
        float rearBrakeScale = 1.f;  // ESC tail-brake reserve
        float tcDriveCap = 3.4e38f;  // traction budget for the driven axle
        std::array<float, 4> absBrakeScale{1.f, 1.f, 1.f, 1.f};
        Vec3 yawTorque;              // ESC yaw damping (world)
        Vec3 lateralForce;           // ESC lateral damping (world)
    };

    void Init(const DriverAssistanceParams& p) { params_ = p; st_ = State{}; }
    // wheels = previous substep state (slip ratios for TC/ABS). rearLoad is the
    // driven-axle load for the TC traction budget.
    Output Step(const CarInput& intent, const CarFrame& frame, float steerAngle,
                const std::array<WheelState, 4>& wheels, float rearLoad,
                const VehicleParams& p, float dt);
    const State& StateRef() const { return st_; }
    void Reset();

private:
    DriverAssistanceParams params_;
    State st_;
};

// --- Driver controls (input processing — NOT a car component) --------------
// Keyboard smoothing + counter-steer/anti-spin. Sits on the input→intent edge.
class DriverControls {
public:
    struct State {
        float steerAngle = 0.f;      // [rad] realized road-wheel angle
        float throttleSmooth = 0.f;
        float brakeSmooth = 0.f;
        bool wingTogglePrev = false;
    };
    struct Output {
        float throttle = 0.f, brake = 0.f;
        float steer = 0.f;           // [rad] realized angle (see State)
        bool handbrake = false, reverse = false;
        bool drsRequest = false, drsZone = false, wingToggle = false, overtake = false;
    };

    void Init(const DriverControlsParams& p) { params_ = p; st_ = State{}; }
    // Smooth raw input (keyboard easing) + counter-steer/anti-spin.
    Output Process(const CarInput& raw, const CarFrame& frame, const VehicleParams& p);
    void Reset();
    const State& StateRef() const { return st_; }

private:
    DriverControlsParams params_;
    State st_;
};

// ============================================================================
// Chassis rigid body + 4 raycast-suspension wheels + combined-slip tires.
// Orchestrates the components; all physics goes through IPhysicsWorld so the
// collision backend is replaceable (qu3e today, Jolt later).
class Vehicle {
public:
    Vehicle() = default;

    void Spawn(IPhysicsWorld& world, const Transform& pose, const VehicleParams& params,
               void* userData = nullptr);
    void Despawn();

    // One fixed physics substep. Orchestrates DriverControls → Suspension →
    // Assistance → Powertrain → Tires → Aero, then applies the summed forces.
    void Update(const CarInput& input, float dt);

    void ProcessContacts(const std::vector<ContactEvent>& contacts, float dt);

    void Reset(const Transform& pose);

    const VehicleState& State() const { return state_; }
    const VehicleParams& Params() const { return params_; }
    BodyHandle Chassis() const { return chassis_; }
    // Part health lives in the aero kit (HUD / snapshot read it from here).
    const AeroKit& Aero() const { return aero_; }

    // Speed-sensitive full-lock road-wheel angle [deg] — shared with the AI.
    float SteerEnvelopeDeg(float speed) const;
    float PartHealth(CarPart part) const {
        return aero_.StateRef().partHealth[static_cast<int>(part)];
    }

    void Render(IRenderer& renderer, const Color& tint) const;
    // Snapshot-friendly draw: no IPhysicsWorld access — used by the async
    // render stage from a published VehicleState + chassis transform.
    static void RenderPose(IRenderer& renderer, const Transform& chassis,
                           const VehicleState& state, const VehicleParams& params,
                           const Color& tint);

private:
    void UpdateDamageFromContact(const ContactEvent& event);

    IPhysicsWorld* world_ = nullptr;
    BodyHandle chassis_ = kInvalidBody;
    VehicleParams params_;
    VehicleState state_;

    // Component assembly
    Engine engine_;
    Transmission gearbox_;
    Differential diff_;
    Suspension susp_;
    AeroKit aero_;
    DriverAssistance assists_;
    DriverControls controls_;
    std::array<TireModel, 4> tires_;

    std::array<Vec3, 4> hardpoints_;  // suspension attach points in local space

    // Mode latch (edge on wingToggle) — orchestrated in Update().
    bool wingTogglePrev_ = false;

    // Per-wheel brake split (N) + rear-axle drive (N), filled each substep.
    std::array<float, 4> brakePerWheel_{};
    float rearAxleDrive_ = 0.f;

    // Powertrain coupling state (engine <-> driveline).
    float drivelineLoad_ = 0.f;  // reflected load the engine stepped against
    int gearPrev_ = 1;           // last gear the crank was rescaled for

    // Visual-only wheel rotation (radians), accumulated from road speed.
    float wheelSpinAngle_ = 0.f;
};


}  // namespace apex
