module;
#include <algorithm>
#include <cstdint>

module rush.engine.physics;
import rush.engine.core;

namespace apex {

namespace {

// Deploy needs a real reserve in the store, not just "not empty": a car that
// spends its last joule per substep would otherwise flicker deploy on/off.
constexpr float kDeployReserve = 0.02f;
// Throttle below this is not a deployment request; brake above it is a regen
// request (both are the smoothed driver values).
constexpr float kDeployThrottle = 0.5f;
constexpr float kRegenBrake = 0.15f;
constexpr float kRegenMinSpeed = 8.f;

}  // namespace

void ErsUnit::Init(const ErsParams& p, std::uint8_t capabilities) {
    params_ = p;
    fitted_ = HasCapability(capabilities, VehicleCapability::Ers);
    overtakeFitted_ = HasCapability(capabilities, VehicleCapability::Overtake);
    Reset();
}

void ErsUnit::Reset() {
    st_ = State{};
    // No store on the car => nothing to charge, ever: the HUD/game read this
    // as "system absent" instead of "empty battery".
    st_.charge = fitted_ ? params_.baseCharge : 0.f;
}

void ErsUnit::Step(const StepInput& in) {
    st_.deploying = false;
    st_.power = 0.f;
    if (!fitted_) {
        st_.charge = 0.f;
        return;
    }

    if (overtakeFitted_ && in.overtake && st_.charge > kDeployReserve &&
        in.throttle > kDeployThrottle) {
        st_.power = params_.deployPower * params_.overtakeMultiplier;
        st_.deploying = true;
        st_.charge = std::max(0.f, st_.charge - st_.power * in.dt / params_.capacity);
    } else if (in.brake > kRegenBrake && in.speed > kRegenMinSpeed) {
        const float regen = params_.regenPower * std::min(1.f, in.brake * 1.4f);
        st_.charge = std::min(1.f, st_.charge + regen * in.dt / params_.capacity);
    }
}

}  // namespace apex
