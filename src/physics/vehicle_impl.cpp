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


#include <algorithm>
#include <cmath>

namespace apex {

namespace {

// Normalized Magic Formula shape in [-1, 1]: peak at s ~ tan(pi/(2C))/By.
float Shape(float stiffness, float shapeC, float e, float s) {
    const float Bs = stiffness * s;
    return std::sin(shapeC * std::atan(Bs - e * (Bs - std::atan(Bs))));
}

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
                                     float brakeForce, float dt, float camberLean) {
    Output out;
    const float r = params_.wheelRadius;
    const float Iw = params_.wheelInertia;

    // Wheel-frame velocity at the contact patch.
    const float vLong = Dot(contactVelocity, forward);
    const float vLat = Dot(contactVelocity, right);

    // Airborne: spin the wheel from drive/brake torque alone (no ground force).
    if (verticalLoad <= 1.f) {
        const float spinSign = std::fabs(omega_) > 0.5f ? Signf(omega_) : Signf(vLong + 1e-3f);
        omega_ += dt * (driveForce * r - brakeForce * r * spinSign) / Iw;
        omega_ = Clampf(omega_, -260.f, 260.f);
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
    const float spinSign = std::fabs(omega_) > 0.5f ? Signf(omega_) : Signf(vLong + 1e-3f);
    const float omegaFree = omega_ + dt * (driveForce * r - brakeForce * r * spinSign) / Iw;
    const float dv = omegaFree * r - vLong;
    const float kappaFree = dv / vRef;

    // Friction ellipse: lateral usage eats longitudinal capacity; deep slip
    // rounds the peak off (a sliding tire loses ~20% of peak).
    const float latUse = std::fabs(shapeY);
    float longCap = peak * std::sqrt(std::max(0.12f, 1.f - 0.85f * latUse * latUse));
    longCap *= 1.f - 0.23f * Clampf((std::fabs(kappaFree) - 0.18f) / 0.67f, 0.f, 1.f);

    float fx = dv / (vRef / (longCap * params_.kStiff) + dt * r * r / Iw);
    fx = Clampf(fx, -longCap, longCap);

    omega_ = omegaFree - dt * fx * r / Iw;
    omega_ = Clampf(omega_, -260.f, 260.f);
    // Service brakes hold the wheel against the rolling direction: no spin
    // reversal while braking forward (or backward) motion.
    if (brakeForce > 1.f && vLong > 0.5f) omega_ = std::max(omega_, 0.f);
    if (brakeForce > 1.f && vLong < -0.5f) omega_ = std::min(omega_, 0.f);

    const float kappa = (omega_ * r - vLong) / vRef;

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

    // Wear accrues from sliding dissipation.
    const float sliding = std::fabs(vLong * kappa) + std::fabs(vLat * tanA);
    out.slipRatio = kappa;
    out.slipAngleDeg = alpha_ * kRadToDeg;
    out.slipMagnitude = std::sqrt(kappa * kappa + tanA * tanA);
    out.wearDelta = params_.wearRate * sliding * dt * (out.slipMagnitude > 0.35f ? 3.f : 1.f);
    wear_ = Clampf(wear_ + out.wearDelta, 0.f, 1.f);

    return out;
}

}  // namespace apex



#include <algorithm>
#include <vector>
#include <cmath>


namespace apex {

namespace {

// Wheel index convention: 0=FL, 1=FR, 2=RL, 3=RR.
constexpr int kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3;

bool IsFront(int wheel) { return wheel < 2; }

// Model space: origin sits on the ground at the wheelbase centre, +Z forward,
// exported by tools/export_car_model.py from the CC0 RaceCar.glb. The body's
// ground plane must meet the wheel contact, i.e. ride height below the chassis
// origin = static suspension length + wheel radius (~0.112 + R).
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

}  // namespace

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
    chassis_ = world.CreateBody(desc).value_or(kInvalidBody);

    // Hardpoints: front axle at +wheelbase/2, rear at -wheelbase/2 (local +Z forward).
    const float halfWb = params.wheelbase * 0.5f;
    const float halfTrack = params.trackWidth * 0.5f;
    const float hardpointY = params.wheelRadius + params.suspensionRestLength * 0.5f;
    hardpoints_[kFrontLeft] = {-halfTrack, 0.f, halfWb};
    hardpoints_[kFrontRight] = {halfTrack, 0.f, halfWb};
    hardpoints_[kRearLeft] = {-halfTrack, 0.f, -halfWb};
    hardpoints_[kRearRight] = {halfTrack, 0.f, -halfWb};
    (void)hardpointY;

    tires_[kFrontLeft] = TireModel(params.frontTire);
    tires_[kFrontRight] = TireModel(params.frontTire);
    tires_[kRearLeft] = TireModel(params.rearTire);
    tires_[kRearRight] = TireModel(params.rearTire);

    for (int i = 0; i < 4; ++i) {
        state_.wheels[i] = WheelState{};
    }
    for (int i = 0; i < static_cast<int>(CarPart::Count); ++i) {
        state_.partHealth[i] = 1.f;
    }

    engineRpm_ = params.idleRpm;
    gear_ = 1;
    escStrength_ = 0.f;
    state_.ersCharge = params.ersBaseCharge;
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
    for (int i = 0; i < static_cast<int>(CarPart::Count); ++i) {
        state_.partHealth[i] = 1.f;
    }
    engineRpm_ = params_.idleRpm;
    gear_ = 1;
    steerAngle_ = 0.f;
    escStrength_ = 0.f;
    state_.ersCharge = params_.ersBaseCharge;
    state_.wingMode = WingMode::ZMode;
}

float Vehicle::SteerEnvelopeDeg(float speed) const {
    // Full lock = 1.2x the kinematic grip angle delta = L * a_max(v) / v^2.
    // The extra 20% is the tire slip-angle headroom the kinematic angle does
    // not include (steering must cover L/R + understeer gradient), so the
    // driver can actually reach the grip limit; the oversteer-gated assists
    // catch anything past it. a_max(v) = mu*g (load-sensitivity margin) +
    // downforce term, derived from the tire / aero calibration.
    const float gripRef = 0.5f * (params_.frontTire.D0 + params_.rearTire.D0);
    const float dfPerV2 = 0.5f * params_.airDensity * params_.downforceArea / params_.mass;
    const float aMax = 9.81f * gripRef * 0.92f + dfPerV2 * speed * speed;
    return std::min(21.f, 1.2f * params_.wheelbase * aMax / (speed * speed + 1.f) * kRadToDeg);
}

void Vehicle::Update(const VehicleInput& input, float dt) {
    if (!world_ || chassis_ == kInvalidBody) return;

    VehicleInput in = input;
    UpdateAssists(in);

    const Transform transform = world_->GetTransform(chassis_).value_or(Transform{});
    const Vec3 velocity = world_->GetLinearVelocity(chassis_).value_or(Vec3{});
    const float speed = Length(velocity);
    const Vec3 forward = TransformDirection(transform, Vec3{0.f, 0.f, 1.f});
    const float forwardSpeed = Dot(velocity, forward);

    // Speed-sensitive steering filter. Keyboard input steps to full lock
    // instantly, so the raw value must never reach the road wheels directly:
    //  - the max angle shrinks with speed toward the grip envelope (~2 deg at
    //    250 km/h),
    //  - both directions ramp gradually (BeamNG-style keyboard filtering):
    //    a key press eases the input in and out instead of stepping it, so a
    //    sudden input change never hits the chassis as a discontinuity.
    //    Wind-up grows slower with speed (a tap at 200 km/h is a gentle
    //    correction); unwind is quicker but still eased so release is smooth.
    // Frame note: the vehicle frame is +Z forward / +Y up (right-handed), so
    // the geometric right side is -X and a right turn is a NEGATIVE yaw around
    // +Y. `in.steer` is defined as positive = steer right, hence the sign.
    const float steerRange = SteerEnvelopeDeg(speed) * kDegToRad;
    float steerTarget = -in.steer * steerRange;

    // Steering assist ("转向与反打辅助"): counter-steer toward the velocity
    // vector — but ONLY on genuine oversteer. Balanced cornering runs a body
    // slip angle of 5-8 deg at the limit (beta = alpha_rear + b/R, physics),
    // so a naive beta ramp fights every hard corner ("转不动弯"). The trigger
    // therefore requires yaw past the steering reference together with slip,
    // or a deep slide on its own; the demand steers toward the velocity
    // vector and damps the yaw error.
    if (params_.steerLevel != AssistLevel::Off && speed > 6.f && !in.handbrake) {
        const float vRight = Dot(velocity, TransformDirection(transform, Vec3{-1.f, 0.f, 0.f}));
        const float beta = std::atan2(vRight, std::max(std::fabs(forwardSpeed), 3.f));
        const float yawRate = Dot(world_->GetAngularVelocity(chassis_).value_or(Vec3{}), Vec3{0.f, 1.f, 0.f});
        const float yawRef = forwardSpeed * std::tan(steerAngle_) / params_.wheelbase;
        const float excess = yawRate - yawRef;
        const bool oversteer = (std::fabs(excess) > 0.2f && std::fabs(beta) > 0.06f) ||
                               std::fabs(beta) > 0.18f;
        if (oversteer) {
            const bool full = params_.steerLevel == AssistLevel::Full;
            const float maxCorr = (full ? 0.85f : 0.45f) * steerRange;
            const float demand = beta / 0.15f + excess / 0.6f;
            steerTarget -= Clampf(demand, -1.f, 1.f) * maxCorr;
        }
    }

    const float tau = std::fabs(steerTarget) < std::fabs(steerAngle_)
                          ? 0.11f
                          : std::min(0.10f + 0.0032f * speed, 0.34f);
    const float steerRate = steerRange / tau * dt;
    if (steerAngle_ < steerTarget) steerAngle_ = std::min(steerAngle_ + steerRate, steerTarget);
    else steerAngle_ = std::max(steerAngle_ - steerRate, steerTarget);

    // Throttle/brake ramp the same way — keyboard keys ease toward full
    // input instead of stepping there (BeamNG-style).
    throttleSmooth_ += (in.throttle - throttleSmooth_) * std::min(1.f, 10.f * dt);
    brakeSmooth_ += (in.brake - brakeSmooth_) * std::min(1.f, 14.f * dt);

    // Wing mode toggle (2026 active aero) — only when the season enables it.
    if (params_.activeAeroEnabled && in.wingToggle && !wingTogglePrev_) {
        state_.wingMode = (state_.wingMode == WingMode::ZMode) ? WingMode::XMode : WingMode::ZMode;
    }
    wingTogglePrev_ = in.wingToggle;

    // DRS: available only inside a map-marked activation zone (map zones,
    // authored on straights) and only when the season enables DRS — no
    // speed or steering gate. Once the driver opens it, it stays open for
    // the whole zone: low speed and turning never close it. Leaving the
    // zone closes it (drsZone goes false); the game resets the toggle
    // latch on exit so re-entry starts closed.
    state_.drsAvailable = params_.drsEnabled && in.drsZone;
    state_.drsActive = in.drsRequest && state_.drsAvailable;

    // ERS: deploy on overtake button when charge remains; regenerate under
    // braking. The manual deploy is the 2026 override mode — off in classic.
    state_.ersDeploying = false;
    float ersPower = 0.f;
    if (params_.overtakeEnabled && in.overtake && state_.ersCharge > 0.02f &&
        throttleSmooth_ > 0.5f) {
        ersPower = params_.ersDeployPower * params_.overtakeMultiplier;
        state_.ersDeploying = true;
        state_.ersCharge = std::max(0.f, state_.ersCharge - ersPower * dt / params_.ersCapacity);
    } else if (brakeSmooth_ > 0.15f && speed > 8.f) {
        const float regen = params_.ersRegenPower * std::min(1.f, brakeSmooth_ * 1.4f);
        state_.ersCharge = std::min(1.f, state_.ersCharge + regen * dt / params_.ersCapacity);
    }

    UpdatePowertrain(in, dt);

    // --- Aero forces -------------------------------------------------------
    // Areas come from the game (damage / DRS / wing mode / pitch balance);
    // this layer only integrates q * A into forces. Unset (-1) falls back to
    // VehicleParams base areas with a pitch balance split so bare-Vehicle
    // unit tests keep a sane aero model without the game layer.
    const float v2 = speed * speed;
    const float dynPressure = 0.5f * params_.airDensity * v2;

    float dragArea = params_.dragArea;
    float frontDfArea = params_.downforceArea * params_.frontDownforceShare;
    float rearDfArea = params_.downforceArea * (1.f - params_.frontDownforceShare);
    const bool gameAero = in.dragArea >= 0.f && in.frontDownforceArea >= 0.f &&
                          in.rearDownforceArea >= 0.f;
    if (gameAero) {
        dragArea = in.dragArea;
        frontDfArea = in.frontDownforceArea;
        rearDfArea = in.rearDownforceArea;
    } else {
        // Pitch shifts aero balance: a nose-down car (braking) moves the
        // center of pressure forward. Nose-up pitch is positive.
        const float pitch = std::asin(
            Clampf(TransformDirection(transform, Vec3{0.f, 0.f, 1.f}).y, -1.f, 1.f));
        const float frontShare =
            Clampf(params_.frontDownforceShare - params_.aeroPitchBalance * pitch, 0.30f, 0.58f);
        frontDfArea = params_.downforceArea * frontShare;
        rearDfArea = params_.downforceArea * (1.f - frontShare);
    }

    const float frontDF = dynPressure * frontDfArea;
    const float rearDF = dynPressure * rearDfArea;
    state_.frontDownforce = frontDF;
    state_.rearDownforce = rearDF;
    state_.dragForce = dynPressure * dragArea;

    // Downforce acts along -world-up through front/rear axles (split by share).
    const Vec3 worldUp{0.f, 1.f, 0.f};
    const float totalDF = frontDF + rearDF;
    if (totalDF > 1.f) {
        const float frontFrac = frontDF / totalDF;
        const Vec3 frontPoint = transform.position + forward * (params_.wheelbase * 0.5f);
        const Vec3 rearPoint = transform.position - forward * (params_.wheelbase * 0.5f);
        (void)world_->ApplyForce(chassis_, -worldUp * frontDF, frontPoint);
        (void)world_->ApplyForce(chassis_, -worldUp * rearDF, rearPoint);
        (void)frontFrac;
    }

    // Drag opposes velocity through the center of pressure.
    if (speed > 0.2f) {
        const Vec3 cop = transform.position + forward * params_.centerOfPressureOffset;
        (void)world_->ApplyForce(chassis_, Normalize(velocity) * -state_.dragForce, cop);
    }

    UpdateWheels(dt);

    // --- Collect state -----------------------------------------------------
    state_.transform = transform;
    state_.velocity = velocity;
    state_.angularVelocity = world_->GetAngularVelocity(chassis_).value_or(Vec3{});
    state_.speed = speed;
    state_.speedKmh = speed * 3.6f;
    state_.forwardSpeed = forwardSpeed;
    state_.rpm = engineRpm_;
    state_.gear = gear_;
    state_.throttle = throttleSmooth_;
    state_.brake = brakeSmooth_;
    state_.steerAngleDeg = steerAngle_ * kRadToDeg;
    state_.wheelSpin = wheelSpinAngle_;

    float damageSum = 0.f;
    for (int i = 0; i < static_cast<int>(CarPart::Count); ++i) damageSum += state_.partHealth[i];
    state_.damageTotal = 1.f - damageSum / static_cast<float>(CarPart::Count);

    wheelSpinAngle_ += (forwardSpeed / params_.wheelRadius) * dt;
}

void Vehicle::UpdatePowertrain(const VehicleInput& input, float dt) {
    const Transform transform = world_->GetTransform(chassis_).value_or(Transform{});
    const Vec3 velocity = world_->GetLinearVelocity(chassis_).value_or(Vec3{});
    const Vec3 forward = TransformDirection(transform, Vec3{0.f, 0.f, 1.f});
    const float forwardSpeed = Dot(velocity, forward);

    // Automatic gearbox (simcade): shift on rpm with a short torque cut.
    const float wheelRadius = params_.wheelRadius;
    const float ratio = params_.gearRatios[static_cast<size_t>(std::max(0, gear_ - 1))];
    const float totalRatio = ratio * params_.finalDrive;

    if (shiftTimer_ > 0.f) {
        shiftTimer_ -= dt;
        driveTorque_ = 0.f;
    } else {
        // rpm from wheel speed (clutched to engine).
        const float wheelRps = forwardSpeed / (2.f * kPi * wheelRadius);
        engineRpm_ = std::max(params_.idleRpm,
                              std::fabs(wheelRps) * 60.f * totalRatio);
        engineRpm_ = std::min(engineRpm_, params_.redlineRpm);

        if (engineRpm_ > params_.shiftUpRpm && gear_ < static_cast<int>(params_.gearRatios.size())) {
            ++gear_;
            shiftTimer_ = params_.shiftTime;
        } else if (engineRpm_ < params_.shiftDownRpm && gear_ > 1) {
            --gear_;
            shiftTimer_ = params_.shiftTime * 0.7f;
        }

        // Torque curve: plateau below peak, taper to redline.
        float torqueFactor = 1.f;
        if (engineRpm_ > params_.peakRpm) {
            const float over = (engineRpm_ - params_.peakRpm) /
                               std::max(1.f, params_.redlineRpm - params_.peakRpm);
            torqueFactor = Clampf(1.f - over * 0.45f, 0.4f, 1.f);
        } else if (engineRpm_ < params_.idleRpm + 800.f) {
            torqueFactor = 0.72f + 0.28f * (engineRpm_ - params_.idleRpm) / 800.f;
        }

        float demand = throttleSmooth_;
        // TC cuts torque when rear wheels are slipping.
        driveTorque_ = demand * params_.maxTorque * torqueFactor;
        if (shiftTimer_ > 0.f) driveTorque_ = 0.f;
    }

    // Reverse gear handling.
    if (input.reverse && forwardSpeed < 0.8f) {
        gear_ = 0;
    } else if (gear_ == 0 && !input.reverse) {
        gear_ = 1;
    }

    const float ratioNow =
        gear_ == 0 ? params_.gearRatios[0] : params_.gearRatios[static_cast<size_t>(std::max(0, gear_ - 1))];
    const float totalRatioNow = ratioNow * params_.finalDrive;

    // Drive force at the rear axle.
    float driveForce = 0.f;
    if (gear_ == 0) {
        driveForce = -throttleSmooth_ * params_.maxTorque * totalRatioNow / wheelRadius * 0.55f;
    } else {
        driveForce = driveTorque_ * totalRatioNow / wheelRadius;
    }

    // TC feed-forward torque budget: cap drive force at the driven axle's
    // traction budget so a lateral reserve survives corner exits (the
    // slip-ratio cut in UpdateAssists handles the transients).
    if (params_.tcLevel != AssistLevel::Off) {
        const float rearLoad = state_.wheels[kRearLeft].load + state_.wheels[kRearRight].load;
        const float reserve = (params_.tcLevel == AssistLevel::Full) ? 0.94f : 1.10f;
        driveForce = std::min(driveForce, std::max(2000.f, rearLoad * params_.rearTire.D0 * reserve));
    }

    // ERS overtake adds force at the same axle.
    if (state_.ersDeploying) {
        const float v = std::max(8.f, Length(world_->GetLinearVelocity(chassis_).value_or(Vec3{})));
        driveForce += params_.ersDeployPower * params_.overtakeMultiplier / v;
    }

    // Engine braking when off throttle.
    float brakeForce = brakeSmooth_ * params_.brakeForceMax;
    if (throttleSmooth_ < 0.05f && gear_ > 0 && std::fabs(Dot(world_->GetLinearVelocity(chassis_).value_or(Vec3{}),
                                                               TransformDirection(world_->GetTransform(chassis_).value_or(Transform{}), Vec3{0, 0, 1}))) > 1.f) {
        brakeForce += params_.engineBrakeTorque * totalRatioNow / wheelRadius * 0.25f;
    }
    if (input.handbrake) brakeForce += params_.brakeForceMax * 0.65f;

    // Split to wheels: drive/brake totals are distributed 50:50 per axle,
    // brake biased 60:40 front like a real single-seater. With ESC on, the
    // rear brake is trimmed while steering so trail-braking never steps the
    // tail out (reference game's rearBrakeReserve).
    const float drivePerRear = driveForce * 0.5f;
    const float brakeFront = brakeForce * 0.6f * 0.5f;
    const float escTrim = (params_.escLevel == AssistLevel::Full)
                              ? 1.f
                              : (params_.escLevel == AssistLevel::Sport ? 0.58f : 0.f);
    const float rearBrakeScale =
        1.f - 0.39f * escTrim * Clampf(std::fabs(steerAngle_) * 5.7f, 0.f, 1.f);
    const float brakeRear = brakeForce * 0.4f * 0.5f * rearBrakeScale;

    drivePerWheel_[kFrontLeft] = 0.f;
    drivePerWheel_[kFrontRight] = 0.f;
    drivePerWheel_[kRearLeft] = drivePerRear;
    drivePerWheel_[kRearRight] = drivePerRear;
    brakePerWheel_[kFrontLeft] = brakeFront;
    brakePerWheel_[kFrontRight] = brakeFront;
    brakePerWheel_[kRearLeft] = brakeRear;
    brakePerWheel_[kRearRight] = brakeRear;
}

void Vehicle::UpdateWheels(float dt) {
    const Transform transform = world_->GetTransform(chassis_).value_or(Transform{});
    const Vec3 up = TransformDirection(transform, Vec3{0.f, 1.f, 0.f});
    const Vec3 forward = TransformDirection(transform, Vec3{0.f, 0.f, 1.f});
    const Vec3 right = TransformDirection(transform, Vec3{1.f, 0.f, 0.f});

    // Simplified camber: static negative camber (top inboard) plus body roll
    // (roll > 0 = leaning left). camber[] holds the wheel-top lean toward the
    // geometric right [rad]; the tire converts it into camber thrust. The
    // camber gain claws back most of the body-roll lean on the compressed
    // side, the rest is what static negative camber is for.
    const float rollF =
        (state_.wheels[kFrontLeft].compression - state_.wheels[kFrontRight].compression) /
        params_.trackWidth;
    const float rollR =
        (state_.wheels[kRearLeft].compression - state_.wheels[kRearRight].compression) /
        params_.trackWidth;
    constexpr float kStaticCamber = 0.035f;  // ~2 deg
    constexpr float kCamberGain = 0.65f;
    const float camberShiftF = (kCamberGain - 1.f) * rollF;
    const float camberShiftR = (kCamberGain - 1.f) * rollR;
    const float camber[4] = {
        kStaticCamber + camberShiftF,
        -kStaticCamber + camberShiftF,
        kStaticCamber + camberShiftR,
        -kStaticCamber + camberShiftR,
    };

    int grassWheels = 0;
    int gravelWheels = 0;
    for (int i = 0; i < 4; ++i) {
        const Vec3 hardpoint = TransformPoint(transform, hardpoints_[i]);
        // Steering: front wheels yaw around the hardpoint.
        Vec3 wheelForward = forward;
        Vec3 wheelRight = right;
        if (IsFront(i)) {
            const Quat steerQ = Quat::FromAxisAngle(up, steerAngle_);
            wheelForward = Rotate(steerQ, forward);
            wheelRight = Rotate(steerQ, right);
        }

        const float maxLen = params_.suspensionRestLength + params_.suspensionMaxTravel;
        const RayHit hit = world_->Raycast(hardpoint, -up, maxLen + params_.wheelRadius, chassis_);

        WheelState& wheel = state_.wheels[i];
        if (hit.hit && Dot(hit.normal, up) > 0.25f) {
            const float distToContact = hit.fraction * (maxLen + params_.wheelRadius);
            const float suspLength = distToContact - params_.wheelRadius;
            const float compression = Clampf(params_.suspensionRestLength - suspLength, -0.02f,
                                             params_.suspensionMaxTravel);
            const float compressionRate =
                (compression - wheel.compression) / std::max(dt, 1e-5f);

            float springForce = params_.springK * std::max(compression, 0.f);
            float damperForce = params_.damperC * compressionRate;
            // Only damp when compressing to avoid jacking.
            if (compressionRate < 0.f) damperForce *= 0.35f;
            // Cap the damper: a raycast miss/hit flicker (wheel grazing a slab
            // edge under body roll) makes the compression rate enormous for a
            // substep and the impulse used to launch the car into the air.
            const float damperCap = params_.springK * 0.30f;
            damperForce = Clampf(damperForce, -damperCap * 0.4f, damperCap);
            float normalForce = std::max(0.f, springForce + damperForce);

            wheel.grounded = true;
            wheel.suspensionLength = suspLength;
            wheel.compression = compression;
            wheel.contactPoint = hit.point;
            wheel.contactNormal = hit.normal;
            wheel.load = normalForce;
            wheel.contactVelocity = world_->GetVelocityAtPoint(chassis_, hit.point).value_or(Vec3{});

            // Suspension force along the strut (chassis up).
            (void)world_->ApplyForce(chassis_, up * normalForce, hit.point);

            // Tire forces in the wheel plane. The contact body carries a
            // SurfaceType tag (road / grass / gravel) so grip per surface is
            // data-driven, not a coordinate hack. ABS releases brake pressure
            // per wheel on lock-up (previous substep's slip ratio — the fresh
            // kappa comes out of Compute).
            float brakeForce = brakePerWheel_[i];
            if (params_.absLevel != AssistLevel::Off && brakeForce > 1.f) {
                const float thresh = (params_.absLevel == AssistLevel::Full) ? 0.115f : 0.18f;
                const float release = (wheel.slipRatio < -thresh)
                                          ? Clampf((-wheel.slipRatio - thresh) * 6.f, 0.f, 0.91f)
                                          : 0.f;
                brakeForce *= 1.f - release;
            }
            const SurfaceType surface = SurfaceFromTag(hit.userData);
            float surfaceGrip = 1.f;
            if (surface == SurfaceType::Grass) {
                surfaceGrip = 0.55f;
                ++grassWheels;
            } else if (surface == SurfaceType::Gravel) {
                surfaceGrip = 0.30f;
                ++gravelWheels;
            }
            const TireModel::Output tire = tires_[i].Compute(
                normalForce * surfaceGrip, wheel.contactVelocity, wheelForward, wheelRight,
                drivePerWheel_[i], brakeForce, dt, camber[i]);
            (void)world_->ApplyForce(chassis_, tire.force, hit.point);
            wheel.force = tire.force;
            wheel.slipRatio = tire.slipRatio;
            wheel.slipAngleDeg = tire.slipAngleDeg;
            wheel.wear = tires_[i].Wear();

            // Tire temperature model: heats with slip work, cools with speed.
            const float slipHeat = std::fabs(tire.slipMagnitude) * normalForce * 1.6e-5f;
            wheel.temperature += slipHeat * dt * 60.f;
            wheel.temperature -= (wheel.temperature - 65.f) * 0.35f * dt;
            wheel.temperature = Clampf(wheel.temperature, 40.f, 160.f);
        } else {
            wheel.grounded = false;
            wheel.suspensionLength = maxLen;
            // Keep the last compression: zeroing it here made the re-contact
            // compression rate huge and spiked the damper (car launch).
            wheel.load = 0.f;
            wheel.force = Vec3{};
        }
    }

    // Runoff drag: grass and especially gravel bleed speed hard (the tire
    // model already loses grip there). Accelerations per wheel so a car fully
    // in the gravel sheds roughly 1g at racing speed — cutting the runoff is
    // never a shortcut.
    if (grassWheels > 0 || gravelWheels > 0) {
        const Vec3 velocity = world_->GetLinearVelocity(chassis_).value_or(Vec3{});
        const float speed = Length(velocity);
        if (speed > 0.3f) {
            float dragAccel = 0.f;
            dragAccel += (0.8f + 0.10f * speed) * static_cast<float>(grassWheels) * 0.25f;
            dragAccel += (1.6f + 0.16f * speed) * static_cast<float>(gravelWheels) * 0.25f;
            (void)world_->ApplyForce(chassis_, Normalize(velocity) * (-dragAccel * params_.mass),
                               transform.position);
        }
    }
}

void Vehicle::UpdateAssists(VehicleInput& input) {
    if (!world_) return;
    const Transform transform = world_->GetTransform(chassis_).value_or(Transform{});
    const Vec3 velocity = world_->GetLinearVelocity(chassis_).value_or(Vec3{});
    // Geometric right side is -X in the vehicle frame (+Z forward / +Y up).
    const Vec3 geoRight = TransformDirection(transform, Vec3{-1.f, 0.f, 0.f});
    const Vec3 up = TransformDirection(transform, Vec3{0.f, 1.f, 0.f});
    const float speed = Length(velocity);
    const float forwardSpeed = Dot(velocity, TransformDirection(transform, Vec3{0.f, 0.f, 1.f}));
    const float vRight = Dot(velocity, geoRight);

    // TC: throttle cut on driven-wheel slip ratio (previous substep's wheel
    // spin state). Slip-based, so straight-line wheelspin is caught too.
    if (params_.tcLevel != AssistLevel::Off) {
        const float slip = std::max(state_.wheels[kRearLeft].slipRatio,
                                    state_.wheels[kRearRight].slipRatio);
        const float thresh = (params_.tcLevel == AssistLevel::Full) ? 0.115f : 0.20f;
        const float cut = Clampf((slip - thresh) * 6.1f, 0.f, 0.94f);
        input.throttle *= (1.f - cut);
    }

    // ABS is applied per wheel in UpdateWheels (it modulates the brake split
    // where the wheel's slip ratio is known).

    // ESC: preventive stability. Steps in on genuine oversteer (same gate as
    // the steering assist — balanced cornering must stay untouched), long
    // before a spin develops. Counter-steer + yaw damping + lateral damping;
    // the rear-brake reserve lives in UpdatePowertrain.
    escStrength_ = 0.f;
    if (params_.escLevel != AssistLevel::Off && !input.handbrake && speed > 5.f) {
        const float beta = std::atan2(vRight, std::max(std::fabs(forwardSpeed), 3.f));
        const float yawRate = Dot(world_->GetAngularVelocity(chassis_).value_or(Vec3{}), up);
        const float yawRef = forwardSpeed * std::tan(steerAngle_) / params_.wheelbase;
        const float excess = yawRate - yawRef;
        const bool full = params_.escLevel == AssistLevel::Full;
        const float strength = full ? 1.f : 0.58f;
        const bool oversteer = (std::fabs(excess) > 0.25f && std::fabs(beta) > 0.06f) ||
                               std::fabs(beta) > (full ? 0.14f : 0.18f);
        if (oversteer) {
            escStrength_ = strength;
            // Small steer trim on top (the steering assist owns the big
            // counter-steer); ESC's main actuators are the yaw torque and
            // lateral damping below.
            input.steer =
                Clampf(input.steer + Clampf(beta * 0.55f + excess * 0.4f, -0.2f, 0.2f) * strength,
                       -1.f, 1.f);
            // Yaw damping about +Y (a right turn is negative yaw). Gains
            // mirror the reference game's 6.6 / 4.3 stabilizer scaled by Iz.
            const float iz = params_.mass * 1.6f;
            (void)world_->ApplyTorque(
                chassis_, Vec3{0.f, -iz * (6.6f * excess + 4.3f * beta) * strength, 0.f});
            // Lateral damping kills the sideways velocity component.
            (void)world_->ApplyForce(
                chassis_, geoRight * (-1.9f * params_.mass * vRight * strength), transform.position);
        }
    }
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

    // Closing-speed damage. Crash energy scales with v^2: a 145 km/h
    // straight-on hit costs over half the part and a 200 km/h wall slam is
    // near-fatal, while scrapes (tiny closing speed) stay harmless. The old
    // linear formula capped at 0.42 per event and counted the full driving
    // speed as impact — every corner scrape hit like a wall crash.
    const float speed01 = Clampf(event.impactSpeed / 40.f, 0.f, 1.5f);
    if (speed01 < 0.05f) return;

    // Barrier hits bruise the car less than car-to-car or ground strikes
    // (the wall's low restitution already scrubs the speed).
    const bool wallHit = SurfaceFromTag(event.userDataA) == SurfaceType::Wall ||
                         SurfaceFromTag(event.userDataB) == SurfaceType::Wall;

    // Region pick: front z > +1.1 => front wing; z < -1.1 => rear wing;
    // |z| <= 1.1 and |x| > 0.55 => sidepods; else floor/floor area.
    CarPart part;
    if (local.z > 1.1f) part = CarPart::FrontWing;
    else if (local.z < -1.15f) part = CarPart::RearWing;
    else if (std::fabs(local.x) > 0.52f) part = CarPart::Sidepods;
    else part = CarPart::Floor;

    const float damage = speed01 * speed01 * (wallHit ? 0.70f : 0.55f);
    const int index = static_cast<int>(part);
    state_.partHealth[index] = Clampf(state_.partHealth[index] - damage, 0.f, 1.f);

    // Side impacts may also clip the floor edge.
    if (part == CarPart::Sidepods && speed01 > 0.35f) {
        const int floorIndex = static_cast<int>(CarPart::Floor);
        state_.partHealth[floorIndex] = Clampf(state_.partHealth[floorIndex] - damage * 0.3f, 0.f, 1.f);
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

    // Simple quality-option shadow: the body footprint squashed onto the
    // ground under the wheels.
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

    // Wheels follow the suspension in Y, steer around up (front) and spin
    // around their axle. Anchor X/Z come from the model so the wheels stay in
    // the arches; Y comes from the live raycast suspension length.
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
