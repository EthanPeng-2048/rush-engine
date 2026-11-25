module;
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include "CarModelData.h"

module rush.engine.physics;
import rush.engine.core;
import rush.engine.render;

namespace apex {

namespace {

// Normalized Magic Formula shape in [-1, 1]: peak at s ~ tan(pi/(2C))/By.
float Shape(float stiffness, float shapeC, float e, float s) {
    const float Bs = stiffness * s;
    return std::sin(shapeC * std::atan(Bs - e * (Bs - std::atan(Bs))));
}

// Wheel index convention: 0=FL, 1=FR, 2=RL, 3=RR.
constexpr int kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3;

bool IsFront(int wheel) { return wheel < 2; }

}  // namespace

TireModel::TireModel(const Params& params) : params_(params) {}

float TireModel::GripMultiplier() const {
    return std::max(params_.minGrip, 1.f - params_.wearGripLoss * wear_);
}

void TireModel::Reset() {
    wear_ = 0.f;
    omega_ = 0.f;
    alpha_ = 0.f;
}

TireModel::Output TireModel::Compute(float verticalLoad, const Vec3& contactVelocity,
                                     const Vec3& forward, const Vec3& right, float driveForce,
                                     float brakeForce, float dt, float camberLean, bool slaved,
                                     float omegaCommanded, float casterTrail) {
    Output out;
    const float r = params_.wheelRadius;
    const float Iw = params_.wheelInertia;

    // Wheel-frame velocity at the contact patch.
    const float vLong = Dot(contactVelocity, forward);
    const float vLat = Dot(contactVelocity, right);

    const float omegaEff = slaved ? omegaCommanded : omega_;

    // Airborne: spin the wheel from drive/brake torque alone (no ground force).
    // Slaved wheels keep the commanded crank speed (rigid driveline).
    if (verticalLoad <= 1.f) {
        if (!slaved) {
            const float spinSign =
                std::fabs(omega_) > 0.5f ? Signf(omega_) : Signf(vLong + 1e-3f);
            omega_ += dt * (driveForce * r - brakeForce * r * spinSign) / Iw;
            omega_ = Clampf(omega_, -260.f, 260.f);
        }
        return out;
    }

    const float vRef = std::max(std::fabs(vLong), 2.f);

    // --- Lateral: transient slip angle (relaxation length) ---------------
    const float alphaTarget = std::atan2(vLat, std::max(std::fabs(vLong), 2.f));
    const float relaxRate = std::min(1.f, dt * std::max(std::fabs(vLong), 4.f) / params_.relaxLength);
    alpha_ += (alphaTarget - alpha_) * relaxRate;
    const float tanA = std::tan(alpha_);

    // --- Peak force with load sensitivity --------------------------------
    const float loadRatio = verticalLoad / params_.Fz0;
    const float peak = params_.D0 * params_.Fz0 *
                       std::pow(std::max(loadRatio, 1e-3f), params_.loadExp) * GripMultiplier();

    const float shapeY = Shape(params_.By, params_.C, params_.E, tanA);

    // --- Longitudinal: implicit wheel-spin dynamics ----------------------
    //   Iw*domega = Tdrive - Tbrake - Fx*r,  Fx ~ kStiff*peak*kappa.
    // Backward Euler yields an unconditionally stable contact damper:
    //   Fx = dv / (vRef/(cap*kStiff) + dt*r^2/Iw)
    // Slaved wheels use the commanded spin speed (crank through the gear);
    // the Iw term then acts as mild driveline compliance/damping.
    const float spinSign = std::fabs(omegaEff) > 0.5f ? Signf(omegaEff) : Signf(vLong + 1e-3f);
    const float omegaFree = slaved ? omegaEff
                                   : omega_ + dt * (driveForce * r - brakeForce * r * spinSign) / Iw;
    const float dv = omegaFree * r - vLong;
    const float kappaFree = dv / vRef;

    // Friction ellipse: lateral usage eats longitudinal capacity; deep slip
    // rounds the peak off (a sliding tire loses ~20% of peak).
    const float latUse = std::fabs(shapeY);
    float longCap = peak * std::sqrt(std::max(0.12f, 1.f - 0.85f * latUse * latUse));
    longCap *= 1.f - 0.23f * Clampf((std::fabs(kappaFree) - 0.18f) / 0.67f, 0.f, 1.f);

    float fx = dv / (vRef / (longCap * params_.kStiff) + dt * r * r / Iw);
    fx = Clampf(fx, -longCap, longCap);

    if (!slaved) {
        omega_ = omegaFree - dt * fx * r / Iw;
        omega_ = Clampf(omega_, -260.f, 260.f);
        // Service brakes hold the wheel against the rolling direction: no spin
        // reversal while braking forward (or backward) motion.
        if (brakeForce > 1.f && vLong > 0.5f) omega_ = std::max(omega_, 0.f);
        if (brakeForce > 1.f && vLong < -0.5f) omega_ = std::min(omega_, 0.f);
    }

    // Reported slip must use the wheel speed actually in effect: for slaved
    // wheels that is the commanded crank speed, not the stale internal spin
    // (which would read a permanent lock). TCS/ESC/ABS consume this.
    const float kappa = (omegaEff * r - vLong) / vRef;

    // --- Lateral force ---------------------------------------------------
    // Longitudinal slip also bleeds lateral grip (combined slip). Force
    // along `right` (+X chassis = geometric left): it opposes vLat and the
    // camber thrust acts toward the wheel-top lean (geometric right is -).
    const float slipBleed = 1.f - 0.55f * Clampf((std::fabs(kappa) - 0.15f) / 0.7f, 0.f, 1.f);
    float fyMax = peak * std::sqrt(std::max(0.11f, 1.f - 0.92f * (fx / peak) * (fx / peak)));
    fyMax *= slipBleed;
    float fyLat = -fyMax * shapeY - params_.camberCoef * verticalLoad * camberLean;
    fyLat = Clampf(fyLat, -1.15f * peak, 1.15f * peak);

    out.force = forward * fx + right * fyLat;

    // Self-aligning yaw moment: lateral force times caster trail. fyLat runs
    // along `right` (+X = geometric left), so a right-hand turn (fyLat < 0)
    // emits a positive (restoring) up-yaw moment — sign convention: right
    // turn is negative yaw, alignment adds positive yaw.
    out.alignMoment = -fyLat * casterTrail;

    // Wear accrues from sliding dissipation.
    const float sliding = std::fabs(vLong * kappa) + std::fabs(vLat * tanA);
    out.slipRatio = kappa;
    out.slipAngleDeg = alpha_ * kRadToDeg;
    out.slipMagnitude = std::sqrt(kappa * kappa + tanA * tanA);
    out.wearDelta = params_.wearRate * sliding * dt * (out.slipMagnitude > 0.35f ? 3.f : 1.f);
    wear_ = Clampf(wear_ + out.wearDelta, 0.f, 1.f);

    return out;
}

// ============================================================================
// Component implementations.

float SteerEnvelopeDeg(const VehicleParams& p, float speed) {
    const float gripRef = 0.5f * (p.frontTire.D0 + p.rearTire.D0);
    const float dfPerV2 = 0.5f * p.aero.airDensity * p.aero.downforceArea / p.mass;
    const float aMax = 9.81f * gripRef * 0.92f + dfPerV2 * speed * speed;
    return std::min(21.f, 1.2f * p.wheelbase * aMax / (speed * speed + 1.f) * kRadToDeg);
}

// --- Engine ---------------------------------------------------------------

float Engine::TorqueFactor() const {
    const float rpm = st_.rpm;
    if (rpm > params_.peakRpm) {
        const float over = (rpm - params_.peakRpm) /
                           std::max(1.f, params_.redlineRpm - params_.peakRpm);
        return Clampf(1.f - over * 0.45f, 0.4f, 1.f);
    }
    // Flat torque plateau below peak (F1-style); the launch needs the engine
    // to out-torque the grip-reflected driveline load or it lugs at the floor.
    return 1.f;
}

float Engine::FrictionTorque(float throttleEff) const {
    const float visc = params_.frictionC0 + params_.frictionC1 * st_.omega;
    const float pump = params_.pumpingTorque * (1.f - throttleEff);
    return visc + pump;
}

Engine::StepOutput Engine::Step(const StepInput& in) {
    StepOutput out;
    const float dt = std::max(1e-5f, in.dt);

    // 1. Intake lag: throttle -> airflow (first-order fill).
    st_.intakeFill += (in.throttle - st_.intakeFill) * std::min(1.f, dt / params_.intakeLag);
    float throttleEff = st_.intakeFill;

    // 2. Rev limiter: hysteretic fuel cut.
    if (st_.fuelCut) {
        if (st_.rpm < params_.fuelCutResumeRpm) st_.fuelCut = false;
    } else if (st_.rpm >= params_.fuelCutRpm) {
        st_.fuelCut = true;
    }
    if (st_.fuelCut) throttleEff = 0.f;

    // 3. Firing phase advance (one firing per 720/cyl deg of crank rotation);
    //    roll the seeded misfire die per firing event.
    const float interval = 4.f * kPi / static_cast<float>(std::max(1, params_.cylinders));
    const float phaseNew = st_.firingPhase + st_.omega * dt / interval;
    const int fired = static_cast<int>(phaseNew) - static_cast<int>(st_.firingPhase);
    st_.firingPhase = phaseNew - std::floor(phaseNew);
    bool misfired = false;
    for (int i = 0; i < fired; ++i) {
        rng_ = rng_ * 1664525u + 1013904223u;
        if (((rng_ >> 8) & 0xFFFFu) / 65535.f < params_.misfireRate) misfired = true;
    }

    // 4. Gas torque: curve * throttle * firing ripple; a misfire kills the stroke.
    const float bump = 0.5f - 0.5f * std::cos(2.f * kPi * st_.firingPhase);
    float gas = params_.maxTorque * TorqueFactor() * throttleEff
              * (1.f + params_.torquePulse * (2.f * bump - 1.f));
    if (misfired) gas *= 0.25f;

    // 5. Idle governor: off-throttle hold at idleRpm; a soft rpm floor keeps
    //    the engine alive at standstill (auto-clutch game behaviour).
    const float friction = FrictionTorque(throttleEff);
    const float idleGap = std::max(0.f, (params_.idleRpm - st_.rpm) / std::max(1.f, params_.idleRpm));
    if (in.throttle < 0.05f && idleGap > 0.f) {
        gas = std::max(gas, friction + params_.idleHoldTorque * idleGap);
    }
    if (st_.rpm < 200.f) {
        st_.omega = 200.f * kPi / 30.f;
        st_.rpm = 200.f;
    }

    // 6. Crank integration against the driveline load; ERS adds crank torque.
    const float ersCrank = in.ersPower > 0.f ? in.ersPower / std::max(6.f, st_.omega) : 0.f;
    const float net = gas - friction - in.loadTorque + ersCrank;
    st_.omega += net / std::max(0.1f, in.inertia) * dt;
    st_.omega = std::max(0.f, st_.omega);
    st_.rpm = st_.omega * 60.f / (2.f * kPi);

    out.crankTorque = gas - friction + ersCrank;
    out.rpm = st_.rpm;
    out.firingPhase = st_.firingPhase;
    out.fuelCut = st_.fuelCut;
    out.misfire = misfired;
    return out;
}

void Engine::Rescale(float factor) {
    st_.omega *= factor;
    st_.rpm = st_.omega * 60.f / (2.f * kPi);
}

float Engine::UpdateErc(const CarInput& intent, float throttle, float brake, float speed,
                        bool overtakeEnabled, float dt) {
    st_.ersDeploying = false;
    float deployPower = 0.f;
    if (overtakeEnabled && intent.overtake && st_.ersCharge > 0.02f && throttle > 0.5f) {
        deployPower = params_.ersDeployPower * params_.overtakeMultiplier;
        st_.ersDeploying = true;
        st_.ersCharge = std::max(0.f, st_.ersCharge - deployPower * dt / params_.ersCapacity);
    } else if (brake > 0.15f && speed > 8.f) {
        const float regen = params_.ersRegenPower * std::min(1.f, brake * 1.4f);
        st_.ersCharge = std::min(1.f, st_.ersCharge + regen * dt / params_.ersCapacity);
    }
    return deployPower;
}

void Engine::Reset() {
    st_ = State{};
    st_.ersCharge = params_.ersBaseCharge;
    rng_ = params_.misfireSeed;
}

// --- Transmission ----------------------------------------------------------

Transmission::Output Transmission::Compute(const CarInput& intent, float crankTorque,
                                           float rpm, float speed, float forwardSpeed,
                                           float wheelRadius, float rearBrakeScale, float dt) {
    Output out;
    const float ratio = params_.gearRatios[static_cast<size_t>(std::max(0, st_.gear - 1))];
    const float totalRatio = ratio * params_.finalDrive;

    // Shift on rpm with a short torque cut (automatic gearbox, simcade). The
    // crank speed rescale for the new ratio happens in the vehicle
    // orchestrator (rigid driveline); here we only cut the delivered torque.
    if (st_.shiftTimer > 0.f) {
        st_.shiftTimer -= dt;
    } else {
        if (rpm > params_.shiftUpRpm && st_.gear < static_cast<int>(params_.gearRatios.size())) {
            ++st_.gear;
            st_.shiftTimer = params_.shiftTime;
        } else if (rpm < params_.shiftDownRpm && st_.gear > 1) {
            --st_.gear;
            st_.shiftTimer = params_.shiftTime * 0.7f;
        }
    }

    // Reverse gear handling.
    if (intent.reverse && forwardSpeed < 0.8f) {
        st_.gear = 0;
    } else if (st_.gear == 0 && !intent.reverse) {
        st_.gear = 1;
    }

    const float ratioNow = params_.gearRatios[static_cast<size_t>(std::max(0, st_.gear - 1))];
    const float totalRatioNow = ratioNow * params_.finalDrive;

    // Drive force at the rear axle: the engine's net crank torque (already
    // throttle-inclusive, engine braking and ERS included) through the gear.
    float driveForce = 0.f;
    if (st_.shiftTimer > 0.f) {
        driveForce = 0.f;
    } else if (st_.gear == 0) {
        driveForce = -crankTorque * totalRatioNow / wheelRadius * 0.55f;
    } else {
        driveForce = crankTorque * totalRatioNow / wheelRadius;
    }

    // Engine braking is inside crankTorque (friction + pumping at the crank),
    // so no separate wheel-side engine-brake term. Service brakes:
    float brakeForce = intent.brake * params_.brakeForceMax;
    if (intent.handbrake) brakeForce += params_.brakeForceMax * 0.65f;

    // Split to wheels: axle-level drive (the Differential splits it between
    // the driven wheels), brake 60:40 front-biased per wheel.
    const float brakeFront = brakeForce * 0.6f * 0.5f;
    const float brakeRear = brakeForce * 0.4f * 0.5f * rearBrakeScale;

    out.rearAxleDrive = driveForce;
    out.brakePerWheel[kFrontLeft] = brakeFront;
    out.brakePerWheel[kFrontRight] = brakeFront;
    out.brakePerWheel[kRearLeft] = brakeRear;
    out.brakePerWheel[kRearRight] = brakeRear;
    return out;
}

void Transmission::Reset() { st_ = State{}; }

// --- Differential -----------------------------------------------------------

Differential::Output Differential::Step(const Input& in) {
    Output out;
    const float r = std::max(in.wheelRadius, 1e-3f);
    const float T_axle = in.axleDrive * r;

    // Power vs braking (engine braking included) parameter sets. Reverse
    // drive is a signed negative force, same as engine braking.
    const bool accel = T_axle > 0.f;
    const float preload = accel ? params_.preloadAccel : params_.preloadDecel;
    const float ramp = accel ? params_.rampAccel : params_.rampDecel;
    const float lock = accel ? params_.lockFactorAccel : params_.lockFactorDecel;

    // Bias law: preload + ramp on the split, capped at lockFactor * |T_axle|.
    const float omegaDiff = 2.f * st_.delta;  // wL - wR
    const float biasMax = lock * std::fabs(T_axle);
    const float biasTarget = Clampf(preload + ramp * omegaDiff, -biasMax, biasMax);

    // Slew limit: a preload step must not slam the bias torque in one
    // substep (avoids oscillation at 240 Hz).
    const float biasStep = params_.biasRateLimit * std::max(in.dt, 1e-5f);
    st_.bias = Clampf(biasTarget, st_.bias - biasStep, st_.bias + biasStep);

    // Split dynamics: the bias torque spins the faster wheel up, the tire
    // reaction difference (in the unsaturated regime) restores it, and the
    // damper bounds it under full spin-up where the reaction difference
    // vanishes (both tires saturated). 2*Iw is the two-wheel inertia about
    // the split degree of freedom. deltaMax is the clutch slip limit: the
    // split cannot outrun the case command.
    const float iw = std::max(in.wheelInertia, 1e-3f);
    const float accelSplit =
        (st_.bias - in.fxTorqueDiff) / (2.f * iw) - params_.deltaDamping * st_.delta;
    st_.delta = Clampf(st_.delta + accelSplit * std::max(in.dt, 1e-5f), -params_.deltaMax,
                       params_.deltaMax);

    const float halfBias = st_.bias * 0.5f;
    out.driveL = (T_axle * 0.5f + halfBias) / r;
    out.driveR = (T_axle * 0.5f - halfBias) / r;
    return out;
}

// --- Suspension ------------------------------------------------------------

void Suspension::Step(const CarFrame& frame, BodyHandle chassis, float wheelRadius,
                      float trackWidth, const std::array<Vec3, 4>& hardpoints, float camber[4],
                      float dt) {
    const Vec3 up = frame.up;
    const float maxLen = params_.suspensionRestLength + params_.suspensionMaxTravel;
    // All four wheel rays share direction and distance (only the hardpoint
    // origins differ): one batch query = one broadphase traversal.
    Vec3 origins[4];
    for (int i = 0; i < 4; ++i) origins[i] = TransformPoint(frame.chassis, hardpoints[i]);
    RayHit hits[4];
    frame.world->RaycastBatch(origins, -up, maxLen + wheelRadius, 4, chassis, hits);

    for (int i = 0; i < 4; ++i) {
        // Static toe (front toeF, rear toeR) rotates the wheel frame around
        // the body up axis. Steer itself is applied by the orchestrator so
        // the ESC-corrected angle reaches the tire model.
        const float toe = IsFront(i) ? params_.toeF : params_.toeR;
        Vec3 wheelForward = frame.forward;
        Vec3 wheelRight = frame.right;
        if (toe != 0.f) {
            const Quat toeQ = Quat::FromAxisAngle(up, toe);
            wheelForward = Rotate(toeQ, frame.forward);
            wheelRight = Rotate(toeQ, frame.right);
        }

        const RayHit& hit = hits[i];

        const float k = IsFront(i) ? params_.springKf : params_.springKr;
        const float c = IsFront(i) ? params_.damperCf : params_.damperCr;
        auto& corner = st_.corner[i];
        if (hit.hit && Dot(hit.normal, up) > 0.25f) {
            const float distToContact = hit.fraction * (maxLen + wheelRadius);
            const float suspLength = distToContact - wheelRadius;
            const float compression = Clampf(params_.suspensionRestLength - suspLength, -0.02f,
                                             params_.suspensionMaxTravel);
            const float compressionRate =
                (compression - corner.compression) / std::max(dt, 1e-5f);

            float springForce = k * std::max(compression, 0.f);
            float damperForce = c * compressionRate;
            // Only damp when compressing to avoid jacking.
            if (compressionRate < 0.f) damperForce *= 0.35f;
            // Cap the damper: a raycast miss/hit flicker makes the compression
            // rate enormous for a substep and used to launch the car into the air.
            const float damperCap = k * 0.30f;
            damperForce = Clampf(damperForce, -damperCap * 0.4f, damperCap);
            const float normalForce = std::max(0.f, springForce + damperForce);

            corner.grounded = true;
            corner.suspensionLength = suspLength;
            corner.compression = compression;
            corner.contactPoint = hit.point;
            corner.contactNormal = hit.normal;
            corner.load = normalForce;
            corner.surface = SurfaceFromTag(hit.userData);
            corner.contactVelocity = frame.world->GetVelocityAtPoint(chassis, hit.point).value_or(Vec3{});
            corner.wheelForward = wheelForward;
            corner.wheelRight = wheelRight;

            // Suspension force along the strut (chassis up).
            (void)frame.world->ApplyForce(chassis, up * normalForce, hit.point);
        } else {
            corner.grounded = false;
            corner.suspensionLength = maxLen;
            corner.load = 0.f;
            corner.contactPoint = Vec3{};
            corner.contactNormal = Vec3{};
            corner.contactVelocity = Vec3{};
            corner.surface = SurfaceType::None;
            corner.wheelForward = wheelForward;
            corner.wheelRight = wheelRight;
        }
    }

    // Anti-roll bars: equal-and-opposite load correction from the axle's
    // compression difference. Total axle load and net force are unchanged;
    // the correction becomes roll moment through the wheel contact offsets,
    // and front/rear bar ratio tunes the roll stiffness split = balance.
    const float arbF = params_.kArbF *
                       (st_.corner[kFrontLeft].compression - st_.corner[kFrontRight].compression);
    st_.corner[kFrontLeft].load -= arbF;
    st_.corner[kFrontRight].load += arbF;
    const float arbR = params_.kArbR *
                       (st_.corner[kRearLeft].compression - st_.corner[kRearRight].compression);
    st_.corner[kRearLeft].load -= arbR;
    st_.corner[kRearRight].load += arbR;
    for (int i = 0; i < 4; ++i) st_.corner[i].load = std::max(0.f, st_.corner[i].load);

    // Simplified camber: static negative camber plus body roll claw-back,
    // per axle.
    const float rollF =
        (st_.corner[kFrontLeft].compression - st_.corner[kFrontRight].compression) / trackWidth;
    const float rollR =
        (st_.corner[kRearLeft].compression - st_.corner[kRearRight].compression) / trackWidth;
    const float camberShiftF = (params_.camberGainF - 1.f) * rollF;
    const float camberShiftR = (params_.camberGainR - 1.f) * rollR;
    camber[0] = params_.camberStaticF + camberShiftF;
    camber[1] = -params_.camberStaticF + camberShiftF;
    camber[2] = params_.camberStaticR + camberShiftR;
    camber[3] = -params_.camberStaticR + camberShiftR;
}

void Suspension::Reset() { st_ = State{}; }

// --- Aero kit --------------------------------------------------------------

void AeroKit::ApplyDamage(CarPart part, float amount) {
    st_.partHealth[static_cast<int>(part)] =
        Clampf(st_.partHealth[static_cast<int>(part)] - amount, 0.f, 1.f);
}

float AeroKit::DamageTotal() const {
    float sum = 0.f;
    for (int i = 0; i < static_cast<int>(CarPart::Count); ++i) sum += st_.partHealth[i];
    return 1.f - sum / static_cast<float>(CarPart::Count);
}

AeroKit::Output AeroKit::Compute(const CarFrame& frame, WingMode wingMode, bool drsActive) {
    const float frontH = st_.partHealth[static_cast<int>(CarPart::FrontWing)];
    const float rearH = st_.partHealth[static_cast<int>(CarPart::RearWing)];
    const float sideH = st_.partHealth[static_cast<int>(CarPart::Sidepods)];
    const float floorH = st_.partHealth[static_cast<int>(CarPart::Floor)];

    const float floorGain = 0.55f + 0.45f * floorH;
    const float sideGain = 0.82f + 0.18f * sideH;

    // Pitch shifts aero balance: a nose-down car (braking) moves the center
    // of pressure forward. Nose-up pitch is positive (body forward.y > 0).
    const float pitch = std::asin(Clampf(frame.forward.y, -1.f, 1.f));
    const float frontShare =
        Clampf(params_.frontDownforceShare - params_.aeroPitchBalance * pitch, 0.30f, 0.58f);

    float frontDfArea = params_.downforceArea * frontShare * (0.35f + 0.65f * frontH) *
                        floorGain * sideGain;
    float rearDfArea = params_.downforceArea * (1.f - frontShare) * (0.30f + 0.70f * rearH) *
                       floorGain * sideGain;
    float dragArea = params_.dragArea * (1.f + 0.35f * (1.f - sideH) + 0.25f * (1.f - frontH) +
                                         0.30f * (1.f - rearH));

    // 2026 X/Z-mode: X trims both downforce and drag, Z is high downforce.
    if (wingMode == WingMode::XMode) {
        frontDfArea *= 0.72f;
        rearDfArea *= 0.78f;
        dragArea *= 0.80f;
    }

    // DRS: rear wing flap opens — big drag cut, rear downforce loss.
    if (drsActive) {
        dragArea *= 0.74f;
        rearDfArea *= 0.55f;
    }

    const float dyn = 0.5f * params_.airDensity * frame.speed * frame.speed;
    st_.frontDownforce = dyn * frontDfArea;
    st_.rearDownforce = dyn * rearDfArea;
    st_.dragForce = dyn * dragArea;

    // Directional stability: body slip beta (along +X = geometric left) and
    // yaw rate feed the aero side force / yaw moment derivatives. The terms
    // scale with q ~ v^2, so they vanish at standstill — no extra gate.
    const float vLat = Dot(frame.velocity, frame.right);
    const float beta = std::atan2(vLat, std::max(std::fabs(frame.forwardSpeed), 3.f));
    const float yawRate = Dot(frame.angularVelocity, frame.up);
    const float qSide = dyn * params_.sideArea;
    const Vec3 sideForce = frame.right * (-qSide * params_.aeroClBeta * beta);
    const float yawMag = -qSide * params_.aeroRefLength *
                         (params_.aeroCnBeta * beta + params_.aeroCnR * yawRate);

    Output out;
    out.frontDF = st_.frontDownforce;
    out.rearDF = st_.rearDownforce;
    out.drag = st_.dragForce;
    out.cop = frame.chassis.position + frame.forward * params_.centerOfPressureOffset +
              frame.up * params_.copHeight;
    out.sideForce = sideForce;
    out.yawMoment = frame.up * yawMag;
    return out;
}

void AeroKit::Reset() { st_ = State{}; }

// --- Driver assistance -----------------------------------------------------

DriverAssistance::Output DriverAssistance::Step(const CarInput& intent, const CarFrame& frame,
                                                float steerAngle,
                                                const std::array<WheelState, 4>& wheels,
                                                float rearLoad, const VehicleParams& p,
                                                float dt) {
    Output out;
    out.throttle = intent.throttle;
    out.steer = steerAngle;
    out.rearBrakeScale = 1.f;
    out.tcDriveCap = 3.4e38f;

    // TCS: throttle cut on driven-wheel slip (previous substep's spin state).
    if (params_.tcLevel != AssistLevel::Off) {
        const float slip = std::max(wheels[kRearLeft].slipRatio, wheels[kRearRight].slipRatio);
        const float thresh =
            (params_.tcLevel == AssistLevel::Full) ? params_.tcThreshFull : params_.tcThreshSport;
        const float cut = Clampf((slip - thresh) * params_.tcCutGain, 0.f, 0.94f);
        out.throttle *= (1.f - cut);
        // TC feed-forward traction budget for the driven axle.
        const float reserve = (params_.tcLevel == AssistLevel::Full)
                                  ? params_.tcBudgetReserveFull
                                  : params_.tcBudgetReserveSport;
        out.tcDriveCap = rearLoad * p.rearTire.D0 * reserve;
    }

    // ABS: release brake pressure per wheel on lock-up.
    if (params_.absLevel != AssistLevel::Off) {
        const float thresh =
            (params_.absLevel == AssistLevel::Full) ? params_.absThreshFull : params_.absThreshSport;
        for (int i = 0; i < 4; ++i) {
            const float release =
                (wheels[i].slipRatio < -thresh)
                    ? Clampf((-wheels[i].slipRatio - thresh) * params_.absReleaseGain, 0.f,
                             params_.absMaxRelease)
                    : 0.f;
            out.absBrakeScale[i] = 1.f - release;
        }
    }

    // ESC rear-brake reserve while steering (independent of oversteer).
    const float escTrim = (params_.escLevel == AssistLevel::Full)
                              ? 1.f
                              : (params_.escLevel == AssistLevel::Sport ? 0.58f : 0.f);
    out.rearBrakeScale =
        1.f - params_.escRearBrakeReserve * escTrim *
                  Clampf(std::fabs(steerAngle) * params_.escRearBrakeGain, 0.f, 1.f);

    // ESC: preventive stability on genuine oversteer — counter-steer trim,
    // yaw damping + lateral damping (the big counter-steer is the controls').
    st_.escStrength = 0.f;
    if (params_.escLevel != AssistLevel::Off && !intent.handbrake && frame.speed > 5.f) {
        // Geometric right is -X = -frame.right (the frame's +X is geometric
        // left); reuse the already-rotated axis instead of another Rotate.
        const Vec3 geoRight = -frame.right;
        const float beta = std::atan2(Dot(frame.velocity, geoRight),
                                      std::max(std::fabs(frame.forwardSpeed), 3.f));
        const float yawRate = Dot(frame.angularVelocity, frame.up);
        const float yawRef = frame.forwardSpeed * std::tan(steerAngle) / p.wheelbase;
        const float excess = yawRate - yawRef;
        const bool full = params_.escLevel == AssistLevel::Full;
        const float strength = full ? params_.escStrengthFull : params_.escStrengthSport;
        const bool oversteer =
            (std::fabs(excess) > params_.escOversteerYaw && std::fabs(beta) > params_.escOversteerBeta) ||
            std::fabs(beta) > (full ? params_.escBetaDeepFull : params_.escBetaDeepSport);
        if (oversteer) {
            st_.escStrength = strength;
            out.steer = Clampf(steerAngle + Clampf(beta * 0.55f + excess * 0.4f, -0.2f, 0.2f) *
                                                   strength,
                               -2.f, 2.f);
            // Yaw damping about +Y (a right turn is negative yaw).
            const float iz = p.mass * 1.6f;
            out.yawTorque = Vec3{0.f,
                                 -iz * (params_.escYawFull * excess + params_.escYawSport * beta) *
                                     strength,
                                 0.f};
            // Lateral damping kills the sideways velocity component.
            out.lateralForce =
                geoRight * (-params_.escLateral * p.mass * Dot(frame.velocity, geoRight) * strength);
        }
    }
    return out;
}

void DriverAssistance::Reset() { st_ = State{}; }

// --- Driver controls (input processing, NOT a car component) ---------------

DriverControls::Output DriverControls::Process(const CarInput& raw, const CarFrame& frame,
                                               const VehicleParams& p) {
    Output out;
    out.handbrake = raw.handbrake;
    out.reverse = raw.reverse;
    out.drsRequest = raw.drsRequest;
    out.drsZone = raw.drsZone;
    out.wingToggle = raw.wingToggle;
    out.overtake = raw.overtake;

    // Speed-sensitive steering filter. Raw keyboard input steps to full lock
    // instantly, so the road-wheel angle must ramp (BeamNG-style easing):
    // wind-up grows slower with speed; unwind is quicker but still eased.
    const float steerRange = SteerEnvelopeDeg(p, frame.speed) * kDegToRad;
    float steerTarget = -raw.steer * steerRange;

    // Counter-steer / anti-spin: steer toward the velocity vector — but ONLY
    // on genuine oversteer (balanced cornering runs a small body slip angle).
    if (params_.steerLevel != AssistLevel::Off && frame.speed > params_.counterSteerMinSpeed &&
        !raw.handbrake) {
        // Geometric right is -X = -frame.right (the frame's +X is geometric
        // left); reuse the already-rotated axis instead of another Rotate.
        const Vec3 geoRight = -frame.right;
        const float vRight = Dot(frame.velocity, geoRight);
        const float beta = std::atan2(vRight, std::max(std::fabs(frame.forwardSpeed), 3.f));
        const float yawRate = frame.angularVelocity.y;
        const float yawRef = frame.forwardSpeed * std::tan(st_.steerAngle) / p.wheelbase;
        const float excess = yawRate - yawRef;
        const bool oversteer =
            (std::fabs(excess) > params_.oversteerYaw && std::fabs(beta) > params_.oversteerBeta) ||
            std::fabs(beta) > params_.oversteerBetaDeep;
        if (oversteer) {
            const bool full = params_.steerLevel == AssistLevel::Full;
            const float maxCorr =
                (full ? params_.counterSteerMaxFull : params_.counterSteerMaxSport) * steerRange;
            const float demand = beta / params_.counterSteerBetaNorm +
                                 excess / params_.counterSteerExcessNorm;
            steerTarget -= Clampf(demand, -1.f, 1.f) * maxCorr;
        }
    }

    const float tau = std::fabs(steerTarget) < std::fabs(st_.steerAngle)
                          ? params_.steerTauUnwind
                          : std::min(params_.steerTauWindBase +
                                         params_.steerTauWindPerSpeed * frame.speed,
                                     params_.steerTauWindMax);
    const float steerRate = steerRange / tau * frame.dt;
    if (st_.steerAngle < steerTarget) st_.steerAngle = std::min(st_.steerAngle + steerRate, steerTarget);
    else st_.steerAngle = std::max(st_.steerAngle - steerRate, steerTarget);

    st_.throttleSmooth += (raw.throttle - st_.throttleSmooth) *
                          std::min(1.f, params_.throttleRate * frame.dt);
    st_.brakeSmooth += (raw.brake - st_.brakeSmooth) *
                       std::min(1.f, params_.brakeRate * frame.dt);

    out.throttle = st_.throttleSmooth;
    out.brake = st_.brakeSmooth;
    out.steer = st_.steerAngle;  // [rad] realized road-wheel angle
    return out;
}

void DriverControls::Reset() { st_ = State{}; }

// ============================================================================
// Vehicle — the component orchestrator.

// Model space: origin sits on the ground at the wheelbase centre, +Z forward,
// exported by tools/export_car_model.py from the CC0 RaceCar.glb.
constexpr float kStaticSuspension = 0.112f;

// The baked model is stubbier than the physics car (wheelbase 3.09 m vs the
// physics 3.6 m). Scale the BODY so the visual footprint matches the physics;
// the wheel mesh keeps the physics radius (0.33 m) and rides the suspension.
constexpr float kBodyScale = 3.6f / 3.088f;

// The source asset's left/right naming mirrors our +X=right convention, so
// anchors are referenced by position: (xSign, zSign) -> physics wheel index.
struct WheelDraw {
    int wheelIndex;
    float anchorX, anchorZ;
};

const WheelDraw kWheelDraws[4] = {
    {kFrontLeft, -0.705f * kBodyScale, 1.544f * kBodyScale},   // x<0, z>0
    {kFrontRight, 0.705f * kBodyScale, 1.544f * kBodyScale},   // x>0, z>0
    {kRearLeft, -0.705f * kBodyScale, -1.544f * kBodyScale},   // x<0, z<0
    {kRearRight, 0.705f * kBodyScale, -1.544f * kBodyScale},   // x>0, z<0
};

// Lazily-built GPU meshes shared by every car (body + one wheel drawn 4x).
struct ModelCache {
    MeshHandle body = kInvalidMesh;
    MeshHandle wheel = kInvalidMesh;
    TextureHandle livery = kInvalidTexture;
};

ModelCache& GetModelCache() {
    static ModelCache cache;
    return cache;
}

MeshData BlobToMesh(const car_model::MeshBlob& blob, float scale = 1.f) {
    MeshData data;
    data.positions.resize(static_cast<size_t>(blob.vertexCount));
    data.normals.resize(static_cast<size_t>(blob.vertexCount));
    for (int i = 0; i < blob.vertexCount; ++i) {
        data.positions[static_cast<size_t>(i)] = {blob.positions[i * 3 + 0] * scale,
                                                  blob.positions[i * 3 + 1] * scale,
                                                  blob.positions[i * 3 + 2] * scale};
        data.normals[static_cast<size_t>(i)] = {blob.normals[i * 3 + 0], blob.normals[i * 3 + 1],
                                                blob.normals[i * 3 + 2]};
    }
    data.uvs.assign(blob.uvs, blob.uvs + blob.vertexCount * 2);
    data.colors.assign(blob.colors, blob.colors + blob.vertexCount * 4);
    data.indices.assign(blob.indices, blob.indices + blob.indexCount);
    return data;
}

void EnsureModel(IRenderer& renderer) {
    ModelCache& cache = GetModelCache();
    if (cache.body == kInvalidMesh) cache.body = renderer.CreateMesh(BlobToMesh(car_model::kBody, kBodyScale));
    if (cache.wheel == kInvalidMesh) cache.wheel = renderer.CreateMesh(BlobToMesh(car_model::kWheel));
    if (cache.livery == kInvalidTexture) {
        TextureData tex;
        tex.width = car_model::kLiveryWidth;
        tex.height = car_model::kLiveryHeight;
        tex.pixels = car_model::kLiveryTexture;
        cache.livery = renderer.CreateTexture(tex);
    }
}

void Vehicle::Spawn(IPhysicsWorld& world, const Transform& pose, const VehicleParams& params,
                    void* userData) {
    world_ = &world;
    params_ = params;

    BodyDesc desc;
    desc.transform = pose;
    desc.halfExtents = {params.chassisHalfX, params.chassisHalfY, params.chassisHalfZ};
    desc.mass = params.mass;
    desc.friction = 0.2f;
    desc.restitution = 0.05f;
    desc.linearDamping = 0.02f;
    desc.angularDamping = 0.25f;
    desc.allowSleep = false;
    desc.userData = userData;

    // Composite collision layout (qu3e/rpc): one tagged box per CarPart plus
    // the untagged main chassis box. The density is derived from the sum of
    // volumes, so the total mass stays exactly `mass` and the inertia tensor
    // follows the real distribution (wide flat body, overhanging wings).
    // nudge ignores shapes and falls back to the single chassisHalf box.
    const auto addBox = [&desc](const PartBoxDesc& b, int partTag) {
        if (b.halfExtents.x <= 0.f || b.halfExtents.y <= 0.f || b.halfExtents.z <= 0.f) return;
        ShapeDesc shape;
        shape.localTransform.position = b.position;  // identity rotation
        shape.halfExtents = b.halfExtents;
        shape.partTag = partTag;
        desc.shapes.push_back(shape);
    };
    for (int p = 0; p < static_cast<int>(CarPart::Count); ++p) {
        if (p == static_cast<int>(CarPart::Sidepods)) {
            // The sidepod entry describes the symmetric +/- offset; expand
            // into two boxes sharing the Sidepods tag.
            const PartBoxDesc& s = params.partBoxes[p];
            if (s.halfExtents.x > 0.f && std::fabs(s.position.x) > 1e-4f) {
                PartBoxDesc l = s, r = s;
                l.position.x = -std::fabs(s.position.x);
                r.position.x = std::fabs(s.position.x);
                addBox(l, p);
                addBox(r, p);
            } else {
                addBox(s, p);
            }
        } else {
            addBox(params.partBoxes[p], p);
        }
    }
    addBox(params.chassisBox, -1);  // main box: no part tag (fallback)

    chassis_ = world.CreateBody(desc).value_or(kInvalidBody);

    // Hardpoints: front axle at +wheelbase/2, rear at -wheelbase/2 (local +Z forward).
    const float halfWb = params.wheelbase * 0.5f;
    const float halfTrack = params.trackWidth * 0.5f;
    hardpoints_[kFrontLeft] = {-halfTrack, 0.f, halfWb};
    hardpoints_[kFrontRight] = {halfTrack, 0.f, halfWb};
    hardpoints_[kRearLeft] = {-halfTrack, 0.f, -halfWb};
    hardpoints_[kRearRight] = {halfTrack, 0.f, -halfWb};

    tires_[kFrontLeft] = TireModel(params.frontTire);
    tires_[kFrontRight] = TireModel(params.frontTire);
    tires_[kRearLeft] = TireModel(params.rearTire);
    tires_[kRearRight] = TireModel(params.rearTire);

    // Component assembly.
    engine_.Init(params.engine);
    gearbox_.Init(params.gearbox);
    diff_.Init(params.differential);
    susp_.Init(params.suspension);
    aero_.Init(params.aero);
    assists_.Init(params.assists);
    controls_.Init(params.controls);

    for (int i = 0; i < 4; ++i) state_.wheels[i] = WheelState{};
    state_.ersCharge = engine_.StateRef().ersCharge;
    state_.wingMode = WingMode::ZMode;
    state_.transform = pose;
}

void Vehicle::Despawn() {
    if (world_ && chassis_ != kInvalidBody) {
        (void)world_->DestroyBody(chassis_);
        chassis_ = kInvalidBody;
    }
    world_ = nullptr;
}

void Vehicle::Reset(const Transform& pose) {
    if (!world_ || chassis_ == kInvalidBody) return;
    (void)world_->SetTransform(chassis_, pose);
    (void)world_->SetLinearVelocity(chassis_, Vec3{});
    (void)world_->SetAngularVelocity(chassis_, Vec3{});
    for (int i = 0; i < 4; ++i) {
        state_.wheels[i].wear = 0.f;
        state_.wheels[i].temperature = 70.f;
        state_.wheels[i].slipRatio = 0.f;
        tires_[i].Reset();
    }
    engine_.Reset();
    gearbox_.Reset();
    diff_.Reset();
    susp_.Reset();
    aero_.Reset();
    assists_.Reset();
    controls_.Reset();
    gearPrev_ = gearbox_.StateRef().gear;
    drivelineLoad_ = 0.f;
    state_.ersCharge = engine_.StateRef().ersCharge;
    state_.wingMode = WingMode::ZMode;
}

float Vehicle::SteerEnvelopeDeg(float speed) const { return apex::SteerEnvelopeDeg(params_, speed); }

void Vehicle::Update(const CarInput& input, float dt) {
    if (!world_ || chassis_ == kInvalidBody) return;

    // --- Build the chassis frame handed to every component -----------------
    CarFrame frame;
    frame.world = world_;
    frame.dt = dt;
    // One batched backend round-trip instead of three handle lookups.
    const ChassisSnapshot snap = world_->FetchChassis(chassis_).value_or(ChassisSnapshot{});
    frame.chassis = snap.transform;
    frame.velocity = snap.linearVelocity;
    frame.speed = Length(frame.velocity);
    frame.forward = TransformDirection(frame.chassis, Vec3{0.f, 0.f, 1.f});
    frame.forwardSpeed = Dot(frame.velocity, frame.forward);
    frame.right = TransformDirection(frame.chassis, Vec3{1.f, 0.f, 0.f});
    frame.up = TransformDirection(frame.chassis, Vec3{0.f, 1.f, 0.f});
    frame.angularVelocity = snap.angularVelocity;

    // --- 1. Driver controls: smoothing + counter-steer (input processing) --
    const DriverControls::Output ctrl = controls_.Process(input, frame, params_);

    // --- Mode latching (wing mode / DRS) ----------------------------------
    if (params_.activeAeroEnabled && ctrl.wingToggle && !wingTogglePrev_) {
        state_.wingMode = (state_.wingMode == WingMode::ZMode) ? WingMode::XMode : WingMode::ZMode;
    }
    wingTogglePrev_ = ctrl.wingToggle;
    state_.drsAvailable = params_.drsEnabled && ctrl.drsZone;
    state_.drsActive = ctrl.drsRequest && state_.drsAvailable;

    // --- 2. Suspension: per-corner loads + contacts -------------------------
    float camber[4];
    susp_.Step(frame, chassis_, params_.wheelRadius, params_.trackWidth, hardpoints_, camber, dt);
    const Suspension::State& susp = susp_.StateRef();
    const float rearLoad = susp.corner[kRearLeft].load + susp.corner[kRearRight].load;

    // --- 3. Driver assistance: TCS / ESC / ABS -----------------------------
    const DriverAssistance::Output aid =
        assists_.Step(input, frame, ctrl.steer, state_.wheels, rearLoad, params_, dt);

    // --- 4. Powertrain: crank dynamics -> torque, gearbox drive/brake split --
    // The engine integrates its own crank speed against the driveline load
    // reflected from the previous substep; the driven wheels are slaved to the
    // crank through the current gear (rigid driveline, tire slip as the only
    // compliance). The crank load is the measured rear-axle reaction (tire
    // force + brake torque): slip feeds back negatively, so braking cannot
    // pull the driveline open (front braking outpacing the commanded wheel
    // speed would flip the rear to drive). The clutch opens only at
    // standstill idle so the engine can hold rpm without shoving the car
    // (auto-clutch behaviour). TCS/ESC enter as throttle cut / brake reserve.
    const float ersPower =
        engine_.UpdateErc(input, ctrl.throttle, ctrl.brake, frame.speed, params_.overtakeEnabled, dt);
    const int gearPre = gearbox_.StateRef().gear;
    const float ratioPre = params_.gearbox.gearRatios[static_cast<size_t>(std::max(0, gearPre - 1))];
    const float totalRatioPre = ratioPre * params_.gearbox.finalDrive;

    const bool clutchOpen =
        gearPre != 0 && std::fabs(frame.forwardSpeed) < 1.f && ctrl.throttle < 0.05f;

    // Gearbox shift reference: the rigid-lock-matched rpm (crank slaved to
    // wheel speed), so a free-revving crank (launch, clutch slip) cannot
    // ratchet the box up through all gears.
    const float matchedRpm =
        std::fabs(frame.forwardSpeed) * totalRatioPre * 60.f / (2.f * kPi * params_.wheelRadius);

    Engine::StepInput ein;
    ein.throttle = gearbox_.StateRef().shiftTimer > 0.f ? 0.f : aid.throttle;
    ein.loadTorque = clutchOpen ? 0.f : drivelineLoad_;
    ein.ersPower = ersPower;
    // Crank + driven-wheel inertias only; the vehicle mass enters through the
    // measured rear-axle reaction (tire force) in loadTorque — reflecting it
    // here as well would double-count it.
    ein.inertia = clutchOpen
                      ? params_.engine.crankInertia
                      : params_.engine.crankInertia +
                            2.f * params_.rearTire.wheelInertia / (totalRatioPre * totalRatioPre);
    ein.dt = dt;
    const Engine::StepOutput eng = engine_.Step(ein);

    const Transmission::Output gb =
        gearbox_.Compute(input, clutchOpen ? 0.f : eng.crankTorque, matchedRpm, frame.speed,
                         frame.forwardSpeed, params_.wheelRadius, aid.rearBrakeScale, dt);

    // Rigid driveline: a gear change rescales the crank speed to the new
    // ratio — but only while engaged; an open clutch leaves the crank free.
    const int gearNow = gearbox_.StateRef().gear;
    if (!clutchOpen && gearNow != gearPrev_) {
        const float ratioPrev = params_.gearbox.gearRatios[static_cast<size_t>(std::max(0, gearPrev_ - 1))];
        const float ratioNew = params_.gearbox.gearRatios[static_cast<size_t>(std::max(0, gearNow - 1))];
        engine_.Rescale((ratioNew * params_.gearbox.finalDrive) /
                        std::max(1e-3f, ratioPrev * params_.gearbox.finalDrive));
        gearPrev_ = gearNow;
    }
    const float totalRatioNow = params_.gearbox.gearRatios[static_cast<size_t>(std::max(0, gearNow - 1))] *
                                params_.gearbox.finalDrive;

    rearAxleDrive_ = gb.rearAxleDrive;
    brakePerWheel_ = gb.brakePerWheel;

    // --- 5. Aero forces ----------------------------------------------------
    const AeroKit::Output aero = aero_.Compute(frame, state_.wingMode, state_.drsActive);
    if (aero.frontDF + aero.rearDF > 1.f) {
        // Downforce along the BODY up axis (not world up): on a banked or
        // pitched surface the load presses the car onto the road correctly.
        // The application points sit at the aero centre height, so drag and
        // downforce keep their pitch moments through the COP offset.
        const Vec3 frontPoint = frame.chassis.position + frame.forward * (params_.wheelbase * 0.5f) +
                                frame.up * params_.aero.copHeight;
        const Vec3 rearPoint = frame.chassis.position - frame.forward * (params_.wheelbase * 0.5f) +
                               frame.up * params_.aero.copHeight;
        (void)world_->ApplyForce(chassis_, -frame.up * aero.frontDF, frontPoint);
        (void)world_->ApplyForce(chassis_, -frame.up * aero.rearDF, rearPoint);
    }
    if (frame.speed > 0.2f) {
        (void)world_->ApplyForce(chassis_, Normalize(frame.velocity) * -aero.drag, aero.cop);
        // Directional stability: the side force acts at the COP (raised by
        // copHeight -> roll moment balanced by the suspension), the yaw
        // moment directly about the body up axis.
        (void)world_->ApplyForce(chassis_, aero.sideForce, aero.cop);
        (void)world_->ApplyTorque(chassis_, aero.yawMoment);
    }

    // --- 6. Wheels: copy suspension + apply tire forces ---------------------
    int grassWheels = 0, gravelWheels = 0;
    float rearFx = 0.f, rearBrake = 0.f;  // driveline load accumulation
    float rearFxL = 0.f, rearFxR = 0.f;   // per-wheel Fx for the diff split dynamics
    float alignMoment = 0.f;              // caster-trail self-aligning yaw
    const float crankOmega = engine_.StateRef().omega;
    const bool reverse = gearNow == 0;
    // Differential case command: the rigid driveline locks the average rear-
    // wheel speed to the crank; the split delta offsets each wheel around it
    // (a spool is delta == 0). With the clutch open the wheels roll free.
    const float caseOmega = (reverse ? -1.f : 1.f) * crankOmega / totalRatioNow;
    const float deltaPrev = diff_.StateRef().delta;
    for (int i = 0; i < 4; ++i) {
        const Suspension::Corner& c = susp.corner[i];
        state_.wheels[i].grounded = c.grounded;
        state_.wheels[i].suspensionLength = c.suspensionLength;
        state_.wheels[i].compression = c.compression;
        state_.wheels[i].load = c.load;
        state_.wheels[i].contactPoint = c.contactPoint;
        state_.wheels[i].contactNormal = c.contactNormal;
        state_.wheels[i].contactVelocity = c.contactVelocity;

        if (!c.grounded) {
            state_.wheels[i].force = Vec3{};
            continue;
        }

        // Wheel frame from the suspension (static toe applied); front wheels
        // add the assistance-corrected steer plus roll steer (front-axle
        // roll → toe change) around the body up axis.
        Vec3 wheelForward = c.wheelForward;
        Vec3 wheelRight = c.wheelRight;
        if (IsFront(i)) {
            const float frontRoll =
                (susp.corner[kFrontLeft].compression - susp.corner[kFrontRight].compression) /
                params_.trackWidth;
            const Quat steerQ =
                Quat::FromAxisAngle(frame.up, aid.steer + params_.suspension.rollSteerF * frontRoll);
            wheelForward = Rotate(steerQ, wheelForward);
            wheelRight = Rotate(steerQ, wheelRight);
        }

        float brakeForce = brakePerWheel_[i];
        if (params_.assists.absLevel != AssistLevel::Off && brakeForce > 1.f) {
            brakeForce *= aid.absBrakeScale[i];
        }

        float surfaceGrip = 1.f;
        if (c.surface == SurfaceType::Grass) {
            surfaceGrip = 0.55f;
            ++grassWheels;
        } else if (c.surface == SurfaceType::Gravel) {
            surfaceGrip = 0.30f;
            ++gravelWheels;
        }

        // Driven wheels are slaved to the differential case at their own
        // commanded spin (case +/- split): the average stays locked to the
        // crank while the LSD split offsets each wheel, so the bias shows up
        // as real per-wheel slip. Drive force is delivered through the crank
        // reaction (drivelineLoad_) exactly like the spool baseline; the
        // Differential only reshapes the spin commands, never the brakes.
        const bool slaved = !IsFront(i) && !clutchOpen;
        const float wheelOmega = (i == kRearLeft) ? caseOmega + deltaPrev : caseOmega - deltaPrev;
        const float trail = IsFront(i) ? params_.suspension.casterTrailF : 0.f;
        const TireModel::Output tire =
            tires_[i].Compute(c.load * surfaceGrip, c.contactVelocity, wheelForward, wheelRight,
                              0.f, brakeForce, dt, camber[i], slaved, wheelOmega, trail);
        (void)world_->ApplyForce(chassis_, tire.force, c.contactPoint);
        state_.wheels[i].force = tire.force;
        state_.wheels[i].slipRatio = tire.slipRatio;
        state_.wheels[i].slipAngleDeg = tire.slipAngleDeg;
        state_.wheels[i].wear = tires_[i].Wear();
        alignMoment += tire.alignMoment;

        // Driveline load: the rear axle's measured reaction — tire force plus
        // applied brake torque. This is the coupling torque the crank must
        // provide; slip feeds back negatively, so braking cannot pull the
        // driveline open (front braking outpacing the commanded wheel speed
        // would flip the rear to drive).
        if (!IsFront(i) && !clutchOpen) {
            const float fxAlong = Dot(tire.force, frame.forward);
            rearFx += fxAlong;
            if (i == kRearLeft) rearFxL = fxAlong;
            else rearFxR = fxAlong;
            rearBrake += brakeForce;
        }

        // Tire temperature: heats with slip work, cools with speed.
        const float slipHeat = std::fabs(tire.slipMagnitude) * c.load * 1.6e-5f;
        state_.wheels[i].temperature += slipHeat * dt * 60.f;
        state_.wheels[i].temperature -= (state_.wheels[i].temperature - 65.f) * 0.35f * dt;
        state_.wheels[i].temperature = Clampf(state_.wheels[i].temperature, 40.f, 160.f);
    }

    // Self-aligning yaw moment from caster trail (front axle only).
    if (alignMoment != 0.f) {
        (void)world_->ApplyTorque(chassis_, frame.up * alignMoment);
    }

    // Differential split dynamics: bias vs tire-reaction difference, with the
    // damper bounding the split under saturated spin-up. Steps AFTER the tire
    // step so the reaction difference is the one just resolved; the updated
    // delta feeds the next substep's commanded spins.
    {
        Differential::Input din;
        din.axleDrive = rearAxleDrive_;
        din.wheelRadius = params_.wheelRadius;
        din.dt = dt;
        din.fxTorqueDiff = (rearFxL - rearFxR) * params_.wheelRadius;
        din.wheelInertia = params_.rearTire.wheelInertia;
        (void)diff_.Step(din);
    }

    // Driveline load reflected to the crank for the next engine step.
    drivelineLoad_ = clutchOpen ? 0.f : (rearFx + rearBrake) * params_.wheelRadius / totalRatioNow;

    // Runoff drag: grass / gravel bleed speed hard.
    if ((grassWheels > 0 || gravelWheels > 0) && frame.speed > 0.3f) {
        float dragAccel = 0.f;
        dragAccel += (0.8f + 0.10f * frame.speed) * static_cast<float>(grassWheels) * 0.25f;
        dragAccel += (1.6f + 0.16f * frame.speed) * static_cast<float>(gravelWheels) * 0.25f;
        (void)world_->ApplyForce(chassis_, Normalize(frame.velocity) * (-dragAccel * params_.mass),
                                 frame.chassis.position);
    }

    // --- 7. ESC actuators ---------------------------------------------------
    if (assists_.StateRef().escStrength > 0.f) {
        (void)world_->ApplyTorque(chassis_, aid.yawTorque);
        (void)world_->ApplyForce(chassis_, aid.lateralForce, frame.chassis.position);
    }

    // --- Collect state ------------------------------------------------------
    state_.transform = frame.chassis;
    state_.velocity = frame.velocity;
    state_.angularVelocity = frame.angularVelocity;
    state_.speed = frame.speed;
    state_.speedKmh = frame.speed * 3.6f;
    state_.forwardSpeed = frame.forwardSpeed;
    state_.rpm = engine_.StateRef().rpm;
    state_.gear = gearbox_.StateRef().gear;
    state_.throttle = ctrl.throttle;
    state_.brake = ctrl.brake;
    state_.steerAngleDeg = aid.steer * kRadToDeg;
    state_.ersCharge = engine_.StateRef().ersCharge;
    state_.ersDeploying = engine_.StateRef().ersDeploying;
    state_.frontDownforce = aero_.StateRef().frontDownforce;
    state_.rearDownforce = aero_.StateRef().rearDownforce;
    state_.dragForce = aero_.StateRef().dragForce;
    state_.wheelSpin = wheelSpinAngle_;
    wheelSpinAngle_ += (frame.forwardSpeed / params_.wheelRadius) * dt;
}

void Vehicle::ProcessContacts(const std::vector<ContactEvent>& contacts, float) {
    for (const ContactEvent& event : contacts) {
        if (event.bodyA == chassis_ || event.bodyB == chassis_) {
            UpdateDamageFromContact(event);
        }
    }
}

void Vehicle::UpdateDamageFromContact(const ContactEvent& event) {
    if (!world_ || !params_.damageEnabled) return;
    const Transform transform = world_->GetTransform(chassis_).value_or(Transform{});

    // Map the world contact point into chassis-local space and pick the part
    // whose region contains it. Local axes: +X right, +Y up, +Z forward.
    const Vec3 local = RotateInv(transform.rotation, event.worldPoint - transform.position);

    // Closing-speed damage. Crash energy scales with v^2; scrapes stay harmless.
    const float speed01 = Clampf(event.impactSpeed / 40.f, 0.f, 1.5f);
    if (speed01 < 0.05f) return;

    const bool wallHit = SurfaceFromTag(event.userDataA) == SurfaceType::Wall ||
                         SurfaceFromTag(event.userDataB) == SurfaceType::Wall;

    // Part tag takes precedence: the tag belongs to THIS car's body (the
    // contact's other side is a wall / ground / other car). Fall back to the
    // coordinate regions when the backend reports no tag (nudge single box)
    // or the hit landed on the untagged main chassis box.
    CarPart part;
    const int selfPart = (event.bodyA == chassis_) ? event.partA : event.partB;
    if (selfPart >= 0 && selfPart < static_cast<int>(CarPart::Count)) {
        part = static_cast<CarPart>(selfPart);
    } else {
        if (local.z > 1.1f) part = CarPart::FrontWing;
        else if (local.z < -1.15f) part = CarPart::RearWing;
        else if (std::fabs(local.x) > 0.52f) part = CarPart::Sidepods;
        else part = CarPart::Floor;
    }

    const float damage = speed01 * speed01 * (wallHit ? 0.70f : 0.55f);
    aero_.ApplyDamage(part, damage);

    // Side impacts may also clip the floor edge.
    if (part == CarPart::Sidepods && speed01 > 0.35f) {
        aero_.ApplyDamage(CarPart::Floor, damage * 0.3f);
    }
}

void Vehicle::Render(IRenderer& renderer, const Color& tint) const {
    if (!world_ || chassis_ == kInvalidBody) return;
    const Transform transform = world_->GetTransform(chassis_).value_or(Transform{});
    RenderPose(renderer, transform, state_, params_, tint);
}

void Vehicle::RenderPose(IRenderer& renderer, const Transform& transform,
                         const VehicleState& state, const VehicleParams& params,
                         const Color& tint) {
    EnsureModel(renderer);
    const ModelCache& cache = GetModelCache();

    // Body: rigidly attached to the chassis, its ground plane at the wheel
    // contact (static ride height below the chassis origin).
    Transform bodyT = transform;
    bodyT.position += TransformDirection(transform, Vec3{0.f, -(kStaticSuspension + params.wheelRadius), 0.f});

    if (renderer.ShadowsEnabled()) {
        bool grounded = false;
        float groundY = 0.f;
        for (const WheelState& wheel : state.wheels) {
            if (!wheel.grounded) continue;
            if (!grounded || wheel.contactPoint.y < groundY) groundY = wheel.contactPoint.y;
            grounded = true;
        }
        if (grounded) {
            renderer.DrawMeshShadow(cache.body, bodyT, groundY, Color{0.f, 0.f, 0.f, 0.32f});
        }
    }

    renderer.DrawMesh(cache.body, bodyT, tint, cache.livery);

    const float steerRad = state.steerAngleDeg * kDegToRad;
    for (const WheelDraw& draw : kWheelDraws) {
        const WheelState& wheel = state.wheels[draw.wheelIndex];
        const float suspLen = wheel.grounded ? wheel.suspensionLength : kStaticSuspension;

        Quat rot = transform.rotation;
        if (IsFront(draw.wheelIndex)) {
            rot = rot * Quat::FromAxisAngle(Vec3{0.f, 1.f, 0.f}, steerRad);
        }
        rot = rot * Quat::FromAxisAngle(Vec3{1.f, 0.f, 0.f}, state.wheelSpin);

        Transform t;
        t.position = TransformPoint(transform, Vec3{draw.anchorX, -suspLen, draw.anchorZ});
        t.rotation = rot;
        const float wearTint = 1.f - wheel.wear * 0.45f;
        renderer.DrawMesh(cache.wheel, t, Color{0.45f, 0.45f, 0.48f, wearTint}, cache.livery);
    }
}

}  // namespace apex
