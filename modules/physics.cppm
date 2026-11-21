module;
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
#include <exstl/errors.hpp>

export module rush.engine.physics;

import rush.engine.core;

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

// Unified physics/collision interface. The vehicle model drives the world
// through this interface only, so qu3e can be swapped for Jolt (CCD, convex
// hulls) without touching gameplay code.
class IPhysicsWorld {
public:
    virtual ~IPhysicsWorld() = default;

    virtual void SetGravity(const Vec3& gravity) = 0;
    virtual void Step(float dt) = 0;

    virtual BodyHandle CreateBody(const BodyDesc& desc) = 0;
    virtual void DestroyBody(BodyHandle body) = 0;

    virtual Transform GetTransform(BodyHandle body) const = 0;
    virtual void SetTransform(BodyHandle body, const Transform& transform) = 0;

    virtual Vec3 GetLinearVelocity(BodyHandle body) const = 0;
    virtual Vec3 GetAngularVelocity(BodyHandle body) const = 0;
    virtual void SetLinearVelocity(BodyHandle body, const Vec3& velocity) = 0;
    virtual void SetAngularVelocity(BodyHandle body, const Vec3& velocity) = 0;
    virtual Vec3 GetVelocityAtPoint(BodyHandle body, const Vec3& worldPoint) const = 0;

    virtual void ApplyForce(BodyHandle body, const Vec3& force, const Vec3& worldPoint) = 0;
    virtual void ApplyImpulse(BodyHandle body, const Vec3& impulse, const Vec3& worldPoint) = 0;
    virtual void ApplyTorque(BodyHandle body, const Vec3& torque) = 0;

    // Raycast against all bodies. `ignore` skips one body (usually the caster).
    virtual RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                           BodyHandle ignore = kInvalidBody) const = 0;

    // Contacts from the most recent Step(). Used for damage part mapping.
    virtual std::vector<ContactEvent> DrainContacts() = 0;
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

    BodyHandle CreateBody(const BodyDesc& desc) override;
    void DestroyBody(BodyHandle body) override;

    Transform GetTransform(BodyHandle body) const override;
    void SetTransform(BodyHandle body, const Transform& transform) override;

    Vec3 GetLinearVelocity(BodyHandle body) const override;
    Vec3 GetAngularVelocity(BodyHandle body) const override;
    void SetLinearVelocity(BodyHandle body, const Vec3& velocity) override;
    void SetAngularVelocity(BodyHandle body, const Vec3& velocity) override;
    Vec3 GetVelocityAtPoint(BodyHandle body, const Vec3& worldPoint) const override;

    void ApplyForce(BodyHandle body, const Vec3& force, const Vec3& worldPoint) override;
    void ApplyImpulse(BodyHandle body, const Vec3& impulse, const Vec3& worldPoint) override;
    void ApplyTorque(BodyHandle body, const Vec3& torque) override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore) const override;

    std::vector<ContactEvent> DrainContacts() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};




// nudge implementation of IPhysicsWorld. Boxes + raycast + contact summaries
// cover the vehicle/track needs; surface tags ride nudge's material palette
// (Material.user_data), so RayHit / ContactEvent report them like qu3e's
// per-body userData. Force/torque accumulate into velocity deltas because
// nudge exposes no force API; rotation sets recreate the body (no setter).
class NudgeWorld final : public IPhysicsWorld {
public:
    NudgeWorld();
    ~NudgeWorld() override;

    void SetGravity(const Vec3& gravity) override;
    void Step(float dt) override;

    BodyHandle CreateBody(const BodyDesc& desc) override;
    void DestroyBody(BodyHandle body) override;

    Transform GetTransform(BodyHandle body) const override;
    void SetTransform(BodyHandle body, const Transform& transform) override;

    Vec3 GetLinearVelocity(BodyHandle body) const override;
    Vec3 GetAngularVelocity(BodyHandle body) const override;
    void SetLinearVelocity(BodyHandle body, const Vec3& velocity) override;
    void SetAngularVelocity(BodyHandle body, const Vec3& velocity) override;
    Vec3 GetVelocityAtPoint(BodyHandle body, const Vec3& worldPoint) const override;

    void ApplyForce(BodyHandle body, const Vec3& force, const Vec3& worldPoint) override;
    void ApplyImpulse(BodyHandle body, const Vec3& impulse, const Vec3& worldPoint) override;
    void ApplyTorque(BodyHandle body, const Vec3& torque) override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore = kInvalidBody) const override;

    std::vector<ContactEvent> DrainContacts() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};


}  // namespace apex
