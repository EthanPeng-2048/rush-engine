module;
#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>
#include <cstdint>
#include <functional>
#include <memory>
#include "nudge.h"

module rush.engine.physics;
import rush.engine.core;

namespace apex {

namespace {

::v3 ToNudge(const Vec3& v) { return V3(v.x, v.y, v.z); }
Vec3 FromNudge(const ::v3& v) { return {v.x, v.y, v.z}; }
::quat ToNudgeQuat(const Quat& q) { return {q.x, q.y, q.z, q.w}; }
Quat FromNudgeQuat(const ::quat& q) { return {q.x, q.y, q.z, q.w}; }

// Diagonal box inertia (body frame) — nudge computes its own internally but
// never exposes it, and the velocity-delta force emulation needs one.
Vec3 InverseInertiaDiag(const BodyDesc& desc) {
    const float m = std::max(desc.mass, 1e-4f);
    const float x = desc.halfExtents.x, y = desc.halfExtents.y, z = desc.halfExtents.z;
    const float ixx = m / 3.f * (y * y + z * z);
    const float iyy = m / 3.f * (x * x + z * z);
    const float izz = m / 3.f * (x * x + y * y);
    return {1.f / std::max(ixx, 1e-4f), 1.f / std::max(iyy, 1e-4f), 1.f / std::max(izz, 1e-4f)};
}

}  // namespace

struct NudgeWorld::Impl {
    struct Entry {
        ::Body body{};
        BodyDesc desc;
        Vec3 invInertia;
        Vec3 forceAccum;
        Vec3 torqueAccum;
    };

    ::World world{};
    bool created = false;
    Vec3 gravity{0.f, -9.81f, 0.f};
    BodyHandle nextHandle = 1;
    std::unordered_map<BodyHandle, Entry> bodies;
    std::unordered_map<uint64_t, BodyHandle> byId;
    std::vector<ContactEvent> contacts;
    std::set<std::pair<uint64_t, uint64_t>> prevPairs;  // rising-edge contact filter

    // Pre-step velocity snapshot: nudge builds contact summaries AFTER the
    // solver has cancelled the approach velocity, so impact speeds have to be
    // measured against the velocities from before the step (qu3e's
    // BeginContact is pre-solve by construction).
    struct PreVel {
        Vec3 v, w, pos;
    };
    std::unordered_map<BodyHandle, PreVel> preVel;

    Vec3 PrePointVelocity(BodyHandle handle, const Vec3& worldPoint) const {
        const auto it = preVel.find(handle);
        if (it == preVel.end()) return Vec3{};
        return it->second.v + Cross(it->second.w, worldPoint - it->second.pos);
    }

    void EnsureWorld() {
        if (created) return;
        ::WorldParams params{};
        params.gravity = ToNudge(gravity);
        params.broadphase = BROADPHASE_BVH;
        params.solver_type = SOLVER_SOFT_STEP;
        params.velocity_iters = 16;  // match the qu3e backend's iteration count
        world = create_world(params);
        world_set_sleep_enabled(world, 0);  // vehicles must never sleep
        created = true;
    }

    BodyHandle handleOf(::Body body) const {
        const auto it = byId.find(body.id);
        return it == byId.end() ? kInvalidBody : it->second;
    }

    // nudge has no force API: fold an impulse into the body velocities
    // directly (v += J/m, w += I^-1 (r x J)) — same first-order integration
    // qu3e performs on its accumulated forces at step time.
    void ApplyImpulseInternal(BodyHandle handle, const Vec3& impulse, const Vec3& worldPoint) {
        const auto it = bodies.find(handle);
        if (it == bodies.end() || it->second.desc.mass <= 0.f) return;
        Entry& entry = it->second;
        const ::quat rot = body_get_rotation(world, entry.body);
        const Quat q = FromNudgeQuat(rot);

        Vec3 v = FromNudge(body_get_velocity(world, entry.body));
        Vec3 w = FromNudge(body_get_angular_velocity(world, entry.body));
        v += impulse / entry.desc.mass;

        const Vec3 r = worldPoint - FromNudge(body_get_position(world, entry.body));
        const Vec3 angular = Cross(r, impulse);                       // world space
        const Vec3 local = RotateInv(q, angular);                     // body space
        w += Rotate(q, Vec3{entry.invInertia.x * local.x, entry.invInertia.y * local.y,
                            entry.invInertia.z * local.z});

        body_set_velocity(world, entry.body, ToNudge(v));
        body_set_angular_velocity(world, entry.body, ToNudge(w));
        body_wake(world, entry.body);
    }

    BodyHandle SpawnNudgeBody(BodyHandle handle, const BodyDesc& desc) {
        EnsureWorld();
        ::BodyParams params{};
        params.position = ToNudge(desc.transform.position);
        params.rotation = ToNudgeQuat(desc.transform.rotation);
        params.mass = desc.mass;
        params.friction = desc.friction;
        params.restitution = desc.restitution;
        params.linear_damping = desc.linearDamping;
        params.angular_damping = desc.angularDamping;
        const ::Body body = create_body(world, params);

        ::ShapeParams shape{};
        shape.type = SHAPE_BOX;
        shape.box.half_extents = ToNudge(desc.halfExtents);
        body_add_shape(world, body, shape);
        body_set_sleep_allowed(world, body, desc.allowSleep ? 1 : 0);

        Entry entry;
        entry.body = body;
        entry.desc = desc;
        entry.invInertia = InverseInertiaDiag(desc);
        bodies[handle] = entry;
        byId[body.id] = handle;
        return handle;
    }
};

NudgeWorld::NudgeWorld() : impl_(std::make_unique<Impl>()) {}

NudgeWorld::~NudgeWorld() {
    if (impl_->created) destroy_world(impl_->world);
}

void NudgeWorld::SetGravity(const Vec3& gravity) {
    // nudge bakes gravity into create_world; must precede the first body.
    impl_->gravity = gravity;
}

void NudgeWorld::Step(float dt) {
    impl_->EnsureWorld();

    // Convert accumulated forces/torques to velocity deltas (qu3e does the
    // same as impulses at step time), then clear the accumulators. The torque
    // arm of a point force was already folded into torqueAccum at ApplyForce
    // time, so the linear part here applies at the center of mass only.
    for (auto& [handle, entry] : impl_->bodies) {
        if (entry.desc.mass <= 0.f) continue;
        const bool hasForce = entry.forceAccum.x != 0.f || entry.forceAccum.y != 0.f ||
                              entry.forceAccum.z != 0.f;
        const bool hasTorque = entry.torqueAccum.x != 0.f || entry.torqueAccum.y != 0.f ||
                               entry.torqueAccum.z != 0.f;
        if (!hasForce && !hasTorque) continue;

        Vec3 v = FromNudge(body_get_velocity(impl_->world, entry.body));
        Vec3 w = FromNudge(body_get_angular_velocity(impl_->world, entry.body));
        if (hasForce) v += entry.forceAccum * (dt / entry.desc.mass);
        if (hasTorque) {
            const Quat q = FromNudgeQuat(body_get_rotation(impl_->world, entry.body));
            const Vec3 local = RotateInv(q, entry.torqueAccum * dt);
            w += Rotate(q, Vec3{entry.invInertia.x * local.x, entry.invInertia.y * local.y,
                                entry.invInertia.z * local.z});
        }
        body_set_velocity(impl_->world, entry.body, ToNudge(v));
        body_set_angular_velocity(impl_->world, entry.body, ToNudge(w));
        body_wake(impl_->world, entry.body);
        entry.forceAccum = Vec3{};
        entry.torqueAccum = Vec3{};
    }

    // Snapshot velocities before the step for impact-speed measurement.
    impl_->preVel.clear();
    for (const auto& [handle, entry] : impl_->bodies) {
        Impl::PreVel pv;
        pv.v = FromNudge(body_get_velocity(impl_->world, entry.body));
        pv.w = FromNudge(body_get_angular_velocity(impl_->world, entry.body));
        pv.pos = FromNudge(body_get_position(impl_->world, entry.body));
        impl_->preVel[handle] = pv;
    }

    world_step(impl_->world, dt);

    // Contact summaries for this step (damage / impact sounds). nudge reports
    // every touching pair every step; qu3e's listener fires once per touch —
    // dedupe to rising edges so a sustained scrape is not N damage events.
    int count = 0;
    const ::ContactSummary* summaries = world_contact_summaries(impl_->world, &count);
    std::set<std::pair<uint64_t, uint64_t>> currentPairs;
    impl_->contacts.clear();
    impl_->contacts.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        const ::ContactSummary& s = summaries[i];
        const auto key = std::make_pair(s.a.id, s.b.id);
        currentPairs.insert(key);
        if (impl_->prevPairs.find(key) != impl_->prevPairs.end()) continue;

        ContactEvent event;
        event.bodyA = impl_->handleOf(s.a);
        event.bodyB = impl_->handleOf(s.b);
        if (event.bodyA == kInvalidBody || event.bodyB == kInvalidBody) continue;
        event.worldPoint = FromNudge(s.point);
        // Summary normal follows the IPhysicsWorld convention (A -> B), same
        // as qu3e's manifold normal. Impact speed must use the PRE-step
        // velocities (see preVel above) or the solver has already cancelled
        // the approach speed and wall slams read as 0 m/s.
        event.worldNormal = FromNudge(s.normal);
        const Vec3 vA = impl_->PrePointVelocity(event.bodyA, event.worldPoint);
        const Vec3 vB = impl_->PrePointVelocity(event.bodyB, event.worldPoint);
        // Closing speed along the normal only (see IPhysicsWorld.h) — a
        // scrape at 250 km/h must not report 70 m/s of "impact".
        event.impactSpeed = std::max(0.f, Dot(vA - vB, event.worldNormal));
        const auto itA = impl_->bodies.find(event.bodyA);
        const auto itB = impl_->bodies.find(event.bodyB);
        event.userDataA = itA != impl_->bodies.end() ? itA->second.desc.userData : nullptr;
        event.userDataB = itB != impl_->bodies.end() ? itB->second.desc.userData : nullptr;
        impl_->contacts.push_back(event);
    }
    impl_->prevPairs.swap(currentPairs);
}

BodyHandle NudgeWorld::CreateBody(const BodyDesc& desc) {
    return impl_->SpawnNudgeBody(impl_->nextHandle++, desc);
}

void NudgeWorld::DestroyBody(BodyHandle handle) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return;
    destroy_body(impl_->world, it->second.body);
    impl_->byId.erase(it->second.body.id);
    impl_->bodies.erase(it);
}

Transform NudgeWorld::GetTransform(BodyHandle handle) const {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return {};
    Transform out;
    out.position = FromNudge(body_get_position(impl_->world, it->second.body));
    out.rotation = FromNudgeQuat(body_get_rotation(impl_->world, it->second.body));
    return out;
}

void NudgeWorld::SetTransform(BodyHandle handle, const Transform& transform) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return;

    // nudge has no rotation setter — recreate the body at the new pose and
    // keep the handle + velocities (Vehicle::Reset zeroes them right after).
    const Vec3 v = GetLinearVelocity(handle);
    const Vec3 w = GetAngularVelocity(handle);
    BodyDesc desc = it->second.desc;
    desc.transform = transform;
    DestroyBody(handle);
    impl_->SpawnNudgeBody(handle, desc);
    SetLinearVelocity(handle, v);
    SetAngularVelocity(handle, w);
}

Vec3 NudgeWorld::GetLinearVelocity(BodyHandle handle) const {
    const auto it = impl_->bodies.find(handle);
    return it == impl_->bodies.end() ? Vec3{}
                                     : FromNudge(body_get_velocity(impl_->world, it->second.body));
}

Vec3 NudgeWorld::GetAngularVelocity(BodyHandle handle) const {
    const auto it = impl_->bodies.find(handle);
    return it == impl_->bodies.end()
               ? Vec3{}
               : FromNudge(body_get_angular_velocity(impl_->world, it->second.body));
}

void NudgeWorld::SetLinearVelocity(BodyHandle handle, const Vec3& velocity) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return;
    body_set_velocity(impl_->world, it->second.body, ToNudge(velocity));
    body_wake(impl_->world, it->second.body);
}

void NudgeWorld::SetAngularVelocity(BodyHandle handle, const Vec3& velocity) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return;
    body_set_angular_velocity(impl_->world, it->second.body, ToNudge(velocity));
    body_wake(impl_->world, it->second.body);
}

Vec3 NudgeWorld::GetVelocityAtPoint(BodyHandle handle, const Vec3& worldPoint) const {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end()) return Vec3{};
    const Vec3 v = FromNudge(body_get_velocity(impl_->world, it->second.body));
    const Vec3 w = FromNudge(body_get_angular_velocity(impl_->world, it->second.body));
    const Vec3 r = worldPoint - FromNudge(body_get_position(impl_->world, it->second.body));
    return v + Cross(w, r);
}

void NudgeWorld::ApplyForce(BodyHandle handle, const Vec3& force, const Vec3& worldPoint) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end() || it->second.desc.mass <= 0.f) return;
    // Accumulate; torque arm folded in now so point-less force paths stay
    // consistent (Step applies force*dt at the origin linearly).
    it->second.forceAccum += force;
    const Vec3 r = worldPoint - FromNudge(body_get_position(impl_->world, it->second.body));
    it->second.torqueAccum += Cross(r, force);
}

void NudgeWorld::ApplyImpulse(BodyHandle handle, const Vec3& impulse, const Vec3& worldPoint) {
    impl_->ApplyImpulseInternal(handle, impulse, worldPoint);
}

void NudgeWorld::ApplyTorque(BodyHandle handle, const Vec3& torque) {
    const auto it = impl_->bodies.find(handle);
    if (it == impl_->bodies.end() || it->second.desc.mass <= 0.f) return;
    it->second.torqueAccum += torque;
}

RayHit NudgeWorld::Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                           BodyHandle ignore) const {
    impl_->EnsureWorld();
    const Vec3 dir = Normalize(direction);
    Vec3 from = origin;
    float traveled = 0.f;

    // world_raycast has no ignore filter: recast past hits on the skipped
    // body (the caster's chassis) until something else is hit.
    for (int attempt = 0; attempt < 8; ++attempt) {
        ::RayHit hit{};
        const float remaining = maxDistance - traveled;
        if (remaining <= 1e-4f) break;
        if (!world_raycast(impl_->world, ToNudge(from), ToNudge(dir), remaining, &hit)) break;

        const BodyHandle handle = impl_->handleOf(hit.body);
        const float advance = std::max(hit.distance, 0.f) + 1e-3f;
        if (handle != kInvalidBody && handle != ignore && hit.distance >= 0.f) {
            RayHit out;
            out.hit = true;
            out.body = handle;
            out.point = FromNudge(hit.point);
            out.normal = FromNudge(hit.normal);
            out.fraction = maxDistance > 1e-6f ? (traveled + hit.distance) / maxDistance : 0.f;
            const auto it = impl_->bodies.find(handle);
            out.userData = it != impl_->bodies.end() ? it->second.desc.userData : nullptr;
            return out;
        }
        from += dir * advance;
        traveled += advance;
    }
    return {};
}

std::vector<ContactEvent> NudgeWorld::DrainContacts() {
    std::vector<ContactEvent> out;
    out.swap(impl_->contacts);
    return out;
}

}  // namespace apex
