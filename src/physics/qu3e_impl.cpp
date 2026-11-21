module;
#include <algorithm>
#include <map>
#include <vector>
#include <cstdint>
#include <functional>
#include <memory>
#include "q3.h"
#include "dynamics/q3Contact.h"

module rush.engine.physics;
import rush.engine.core;

namespace apex {

namespace {

q3Vec3 ToQ3(const Vec3& v) { return {v.x, v.y, v.z}; }
Vec3 FromQ3(const q3Vec3& v) { return {v.x, v.y, v.z}; }

q3Transform ToQ3Transform(const Transform& t) {
    q3Transform out;
    out.position = ToQ3(t.position);
    // q3Mat3 columns ex/ey/ez from quaternion.
    const float x = t.rotation.x, y = t.rotation.y, z = t.rotation.z, w = t.rotation.w;
    out.rotation.ex = {1.f - 2.f * (y * y + z * z), 2.f * (x * y + z * w), 2.f * (x * z - y * w)};
    out.rotation.ey = {2.f * (x * y - z * w), 1.f - 2.f * (x * x + z * z), 2.f * (y * z + x * w)};
    out.rotation.ez = {2.f * (x * z + y * w), 2.f * (y * z - x * w), 1.f - 2.f * (x * x + y * y)};
    return out;
}

Transform FromQ3Transform(const q3Transform& t, const q3Quaternion& q) {
    Transform out;
    out.position = FromQ3(t.position);
    out.rotation = {q.x, q.y, q.z, q.w};
    return out;
}

}  // namespace

// Captures contacts produced during the most recent scene step.
struct Qu3eWorld::Impl final : public q3ContactListener {
public:
    struct PendingForce {
        BodyHandle body = kInvalidBody;
        Vec3 force;
        Vec3 point;
    };

    explicit Impl(float stepDt)
        // q3Scene advances by the fixed dt given here on every Step(). The
        // game calls Step() once per physics substep, so use the same rate.
        : scene(stepDt, q3Vec3{0.f, -9.81f, 0.f}, 12) {}

    q3Scene scene;
    std::map<BodyHandle, q3Body*> bodies;
    BodyHandle nextHandle = 1;
    std::vector<ContactEvent> contacts;
    std::map<const q3Box*, BodyHandle> boxToBody;
    std::vector<PendingForce> pendingForces;

    void BeginContact(const q3ContactConstraint* contact) override {
        if (!contact || contact->manifold.contactCount <= 0) return;
        const q3Body* a = contact->bodyA;
        const q3Body* b = contact->bodyB;
        if (!a || !b) return;

        ContactEvent event;
        event.bodyA = handleOf(a);
        event.bodyB = handleOf(b);
        if (event.bodyA == 0 || event.bodyB == 0) return;

        // q3Contact::position is already a world-space contact point.
        event.worldPoint = FromQ3(contact->manifold.contacts[0].position);
        event.worldNormal = FromQ3(contact->manifold.normal);
        // Closing speed along the normal only. Full relative velocity counts
        // the driving speed as "impact" — a floor scrape at 250 km/h used to
        // report 70 m/s and blow half the car's health off in a corner.
        const Vec3 point = event.worldPoint;
        const Vec3 rel = FromQ3(a->GetVelocityAtWorldPoint(ToQ3(point)) -
                                b->GetVelocityAtWorldPoint(ToQ3(point)));
        // normal runs A -> B, so Dot(rel, n) > 0 means A closes on B.
        event.impactSpeed = std::max(0.f, Dot(rel, event.worldNormal));
        event.userDataA = const_cast<void*>(a->GetUserData());
        event.userDataB = const_cast<void*>(b->GetUserData());
        contacts.push_back(event);
    }

    void EndContact(const q3ContactConstraint*) override {}

    BodyHandle handleOf(const q3Body* body) const {
        for (const auto& [handle, ptr] : bodies) {
            if (ptr == body) return handle;
        }
        return 0;
    }
};

namespace {

// q3Scene::RayCast performs the box intersection test before invoking
// ReportShape and writes the hit into the shared q3RaycastData, so the
// callback can read the hit directly and only has to pick the closest one.
class ClosestHitCallback final : public q3QueryCallback {
public:
    const q3Scene* scene = nullptr;
    q3RaycastData* ray = nullptr;
    float maxDistance = 0.f;
    BodyHandle ignore = kInvalidBody;
    const std::map<const q3Box*, BodyHandle>* boxToBody = nullptr;
    RayHit result;

    bool ReportShape(q3Box* box) override {
        const auto it = boxToBody->find(box);
        if (it != boxToBody->end() && it->second == ignore) return true;

        // `ray->toi` / `ray->normal` were just written by the scene's test.
        const float t = ray->toi;
        // A ray that starts inside a box reports toi=0 with an uninitialized
        // normal (q3Box::Raycast never writes n0 when all entry t0 < 0), so
        // those hits are untrustworthy — skip them.
        if (t <= 0.f) return true;
        if (!result.hit || t < ray->t) {
            ray->t = t;  // prune anything farther than the best hit so far
            result.hit = true;
            result.point = FromQ3(ray->GetImpactPoint());
            result.normal = FromQ3(ray->normal);
            result.fraction = maxDistance > 1e-6f ? t / maxDistance : 0.f;
            result.body = it != boxToBody->end() ? it->second : kInvalidBody;
            result.userData = box->GetUserdata();
        }
        return true;
    }
};

}  // namespace

Qu3eWorld::Qu3eWorld(float stepDt) : impl_(new Impl(stepDt)) {
    impl_->scene.SetContactListener(impl_.get());
    impl_->scene.SetAllowSleep(false);
    impl_->scene.SetIterations(16);
}

Qu3eWorld::~Qu3eWorld() = default;

void Qu3eWorld::SetGravity(const Vec3& gravity) { impl_->scene.SetGravity(ToQ3(gravity)); }

void Qu3eWorld::Step(float dt) {
    for (const Impl::PendingForce& pf : impl_->pendingForces) {
        const auto it = impl_->bodies.find(pf.body);
        if (it == impl_->bodies.end()) continue;
        it->second->ApplyLinearImpulseAtWorldPoint(ToQ3(pf.force * dt), ToQ3(pf.point));
    }
    impl_->pendingForces.clear();
    (void)dt;  // q3Scene advances by the dt fixed at construction (1/240).
    impl_->scene.Step();
    // Async: publish contacts to CollisionBus; consumers Drain on their stage.
    for (const ContactEvent& e : impl_->contacts) CollisionBus().Publish(e);
    impl_->contacts.clear();
}

BodyHandle Qu3eWorld::CreateBody(const BodyDesc& desc) {
    q3BodyDef def;
    def.position = ToQ3(desc.transform.position);
    // q3BodyDef takes axis+angle; convert from quaternion.
    const Quat& q = desc.transform.rotation;
    const float sinHalf = std::sqrt(std::max(0.f, 1.f - q.w * q.w));
    if (sinHalf > 1e-5f) {
        def.axis = q3Vec3{q.x / sinHalf, q.y / sinHalf, q.z / sinHalf};
        def.angle = 2.f * std::atan2(sinHalf, q.w);
    }
    def.bodyType = desc.mass > 0.f ? eDynamicBody : eStaticBody;
    def.linearDamping = desc.linearDamping;
    def.angularDamping = desc.angularDamping;
    def.allowSleep = desc.allowSleep;
    def.userData = desc.userData;

    q3Body* body = impl_->scene.CreateBody(def);

    q3BoxDef boxDef;
    q3Transform local;
    q3Identity(local);
    // q3BoxDef::Set takes FULL extents and halves them internally (q3Box.inl:
    // m_e = extents * 0.5). Passing half-extents would shrink every collider
    // by 2x (and mass by 8x).
    boxDef.Set(local, ToQ3(desc.halfExtents * 2.f));
    boxDef.SetFriction(desc.friction);
    boxDef.SetRestitution(desc.restitution);
    if (desc.mass > 0.f) {
        boxDef.SetDensity(desc.mass / std::max(0.001f, 8.f * desc.halfExtents.x * desc.halfExtents.y *
                                                            desc.halfExtents.z));
    }
    const q3Box* box = body->AddBox(boxDef);
    impl_->boxToBody[box] = impl_->nextHandle;
    impl_->bodies[impl_->nextHandle] = body;
    return impl_->nextHandle++;
}

void Qu3eWorld::DestroyBody(BodyHandle handle) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return;
    impl_->scene.RemoveBody(it->second);
    impl_->bodies.erase(it);
}

Transform Qu3eWorld::GetTransform(BodyHandle handle) const {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return {};
    return FromQ3Transform(it->second->GetTransform(), it->second->GetQuaternion());
}

void Qu3eWorld::SetTransform(BodyHandle handle, const Transform& transform) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return;
    const Quat& q = transform.rotation;
    const float sinHalf = std::sqrt(std::max(0.f, 1.f - q.w * q.w));
    if (sinHalf > 1e-5f) {
        it->second->SetTransform(ToQ3(transform.position),
                                 q3Vec3{q.x / sinHalf, q.y / sinHalf, q.z / sinHalf},
                                 2.f * std::atan2(sinHalf, q.w));
    } else {
        it->second->SetTransform(ToQ3(transform.position));
    }
}

Vec3 Qu3eWorld::GetLinearVelocity(BodyHandle handle) const {
    const auto it = impl_->bodies.find(handle);
    return it == impl_->bodies.end() ? Vec3{} : FromQ3(it->second->GetLinearVelocity());
}

Vec3 Qu3eWorld::GetAngularVelocity(BodyHandle handle) const {
    const auto it = impl_->bodies.find(handle);
    return it == impl_->bodies.end() ? Vec3{} : FromQ3(it->second->GetAngularVelocity());
}

void Qu3eWorld::SetLinearVelocity(BodyHandle handle, const Vec3& velocity) {
    const auto it = impl_->bodies.find(handle);
    if (it != impl_->bodies.end()) it->second->SetLinearVelocity(ToQ3(velocity));
}

void Qu3eWorld::SetAngularVelocity(BodyHandle handle, const Vec3& velocity) {
    const auto it = impl_->bodies.find(handle);
    if (it != impl_->bodies.end()) it->second->SetAngularVelocity(ToQ3(velocity));
}

Vec3 Qu3eWorld::GetVelocityAtPoint(BodyHandle handle, const Vec3& worldPoint) const {
    const auto it = impl_->bodies.find(handle);
    return it == impl_->bodies.end() ? Vec3{} : FromQ3(it->second->GetVelocityAtWorldPoint(ToQ3(worldPoint)));
}

void Qu3eWorld::ApplyForce(BodyHandle handle, const Vec3& force, const Vec3& worldPoint) {
    // Buffer and convert to an impulse in Step(): qu3e's ApplyForceAtWorldPoint
    // treats its argument as a mass-scaled acceleration (m_force += force *
    // mass) while leaving the torque term unscaled, so passing Newtons makes
    // linear response mass-independent and explodes. Impulses have clean
    // semantics on both axes.
    impl_->pendingForces.push_back({handle, force, worldPoint});
}

void Qu3eWorld::ApplyImpulse(BodyHandle handle, const Vec3& impulse, const Vec3& worldPoint) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return;
    it->second->ApplyLinearImpulseAtWorldPoint(ToQ3(impulse), ToQ3(worldPoint));
}

void Qu3eWorld::ApplyTorque(BodyHandle handle, const Vec3& torque) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return;
    it->second->ApplyTorque(ToQ3(torque));
}

RayHit Qu3eWorld::Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                          BodyHandle ignore) const {
    q3RaycastData ray;
    // dir is normalized, so t is the arc length to the endpoint.
    ray.Set(ToQ3(origin), ToQ3(Normalize(direction)), maxDistance);

    ClosestHitCallback callback;
    callback.scene = &impl_->scene;
    callback.ray = &ray;
    callback.maxDistance = maxDistance;
    callback.ignore = ignore;
    callback.boxToBody = &impl_->boxToBody;
    impl_->scene.RayCast(&callback, ray);
    return callback.result;
}

std::vector<ContactEvent> Qu3eWorld::DrainContacts() {
    std::vector<ContactEvent> out;
    CollisionBus().Drain(out);
    return out;
}

}  // namespace apex
