module;
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <functional>
#include <memory>
#include <exstl/error_macros.hpp>
#include "rpc.h"
#include "dynamics/rpcContact.h"

module rush.engine.physics;
import rush.engine.core;

namespace apex {

namespace {

rpcVec3 ToRpc(const Vec3& v) { return {v.x, v.y, v.z}; }
Vec3 FromRpc(const rpcVec3& v) { return {v.x, v.y, v.z}; }

rpcTransform ToRpcTransform(const Transform& t) {
    rpcTransform out;
    out.position = ToRpc(t.position);
    // rpcMat3 columns ex/ey/ez from quaternion.
    const float x = t.rotation.x, y = t.rotation.y, z = t.rotation.z, w = t.rotation.w;
    out.rotation.ex = {1.f - 2.f * (y * y + z * z), 2.f * (x * y + z * w), 2.f * (x * z - y * w)};
    out.rotation.ey = {2.f * (x * y - z * w), 1.f - 2.f * (x * x + z * z), 2.f * (y * z + x * w)};
    out.rotation.ez = {2.f * (x * z + y * w), 2.f * (y * z - x * w), 1.f - 2.f * (x * x + y * y)};
    return out;
}

Transform FromRpcTransform(const rpcTransform& t, const rpcQuaternion& q) {
    Transform out;
    out.position = FromRpc(t.position);
    out.rotation = {q.x, q.y, q.z, q.w};
    return out;
}

// Real error channel for the RPC adapter (unlike qu3e/nudge compat layers).
rush::Result<rpcBody*> FindBody(const std::unordered_map<BodyHandle, rpcBody*>& bodies, BodyHandle handle) {
    const auto it = bodies.find(handle);
    if (it == bodies.end() || !it->second) {
        EX_FAIL("rpc: invalid body handle " + std::to_string(handle));
    }
    return it->second;
}

rush::Result<rpcBox*> FindBox(const std::unordered_map<BodyHandle, rpcBox*>& boxes, BodyHandle handle) {
    const auto it = boxes.find(handle);
    if (it == boxes.end() || !it->second) {
        EX_FAIL("rpc: invalid body handle " + std::to_string(handle));
    }
    return it->second;
}

rush::Result<void> ValidateBodyDesc(const BodyDesc& desc) {
    const Vec3 he = desc.halfExtents;
    if (!std::isfinite(he.x) || !std::isfinite(he.y) || !std::isfinite(he.z) || he.x <= 0.f ||
        he.y <= 0.f || he.z <= 0.f) {
        EX_FAIL("rpc: CreateBody halfExtents must be finite and > 0");
    }
    if (!std::isfinite(desc.mass) || desc.mass < 0.f) {
        EX_FAIL("rpc: CreateBody mass must be finite and >= 0");
    }
    if (!std::isfinite(desc.friction) || !std::isfinite(desc.restitution)) {
        EX_FAIL("rpc: CreateBody friction/restitution must be finite");
    }
    if (!std::isfinite(desc.transform.position.x) || !std::isfinite(desc.transform.position.y) ||
        !std::isfinite(desc.transform.position.z)) {
        EX_FAIL("rpc: CreateBody position must be finite");
    }
    return {};
}

}  // namespace

// Captures contacts produced during the most recent scene step.
struct RushPhysicsCore::Impl final : public rpcContactListener {
public:
    struct PendingForce {
        BodyHandle body = kInvalidBody;
        Vec3 force;
        Vec3 point;
    };

    explicit Impl(float stepDt)
        // rpcScene advances by the fixed dt given here on every Step(). The
        // game calls Step() once per physics substep, so use the same rate.
        : scene(stepDt, rpcVec3{0.f, -9.81f, 0.f}, 12) {}

    rpcScene scene;
    std::unordered_map<BodyHandle, rpcBody*> bodies;
    std::unordered_map<BodyHandle, rpcBox*> bodyBoxes;
    BodyHandle nextHandle = 1;
    std::vector<ContactEvent> contacts;
    std::unordered_map<const rpcBox*, BodyHandle> boxToBody;
    std::vector<PendingForce> pendingForces;

    void BeginContact(const rpcContactConstraint* contact) override {
        if (!contact || contact->manifold.contactCount <= 0) return;
        const rpcBody* a = contact->bodyA;
        const rpcBody* b = contact->bodyB;
        if (!a || !b) return;

        // O(1) reverse lookup via the box→handle map (each adapter body owns
        // exactly one box). Never scan the body table on the contact path.
        const auto itA = boxToBody.find(contact->A);
        const auto itB = boxToBody.find(contact->B);
        if (itA == boxToBody.end() || itB == boxToBody.end()) return;

        ContactEvent event;
        event.bodyA = itA->second;
        event.bodyB = itB->second;

        // rpcContact::position is already a world-space contact point.
        event.worldPoint = FromRpc(contact->manifold.contacts[0].position);
        event.worldNormal = FromRpc(contact->manifold.normal);
        // Closing speed along the normal only. Full relative velocity counts
        // the driving speed as "impact" — a floor scrape at 250 km/h used to
        // report 70 m/s and blow half the car's health off in a corner.
        const Vec3 point = event.worldPoint;
        const Vec3 rel = FromRpc(a->GetVelocityAtWorldPoint(ToRpc(point)) -
                                b->GetVelocityAtWorldPoint(ToRpc(point)));
        // normal runs A -> B, so Dot(rel, n) > 0 means A closes on B.
        event.impactSpeed = std::max(0.f, Dot(rel, event.worldNormal));
        event.userDataA = const_cast<void*>(a->GetUserData());
        event.userDataB = const_cast<void*>(b->GetUserData());
        // Shape tags: contact->A/B are the actual boxes, so the per-part tag
        // rides on the box userdata (value + 1: CarPart 0 != "no tag").
        const auto tagOf = [](const rpcBox* box) {
            return box && box->GetUserdata()
                       ? static_cast<int>(reinterpret_cast<uintptr_t>(box->GetUserdata())) - 1
                       : -1;
        };
        event.partA = tagOf(contact->A);
        event.partB = tagOf(contact->B);
        contacts.push_back(event);
    }

    void EndContact(const rpcContactConstraint*) override {}
};

namespace {

// rpcScene::RayCast performs the box intersection test before invoking
// ReportShape and writes the hit into the shared rpcRaycastData, so the
// callback can read the hit directly and only has to pick the closest one.
class ClosestHitCallback final : public rpcQueryCallback {
public:
    const rpcScene* scene = nullptr;
    rpcRaycastData* ray = nullptr;
    float maxDistance = 0.f;
    BodyHandle ignore = kInvalidBody;
    const std::unordered_map<const rpcBox*, BodyHandle>* boxToBody = nullptr;
    RayHit result;

    bool ReportShape(rpcBox* box) override {
        const auto it = boxToBody->find(box);
        if (it != boxToBody->end() && it->second == ignore) return true;

        // `ray->toi` / `ray->normal` were just written by the scene's test.
        const float t = ray->toi;
        // A ray that starts inside a box reports toi=0 with an uninitialized
        // normal (rpcBox::Raycast never writes n0 when all entry t0 < 0), so
        // those hits are untrustworthy — skip them.
        if (t <= 0.f) return true;
        if (!result.hit || t < ray->t) {
            ray->t = t;  // prune anything farther than the best hit so far
            result.hit = true;
            result.point = FromRpc(ray->GetImpactPoint());
            result.normal = FromRpc(ray->normal);
            result.fraction = maxDistance > 1e-6f ? t / maxDistance : 0.f;
            result.body = it != boxToBody->end() ? it->second : kInvalidBody;
            result.userData = box->GetUserdata();
            result.friction = box->friction;
        }
        return true;
    }
};

}  // namespace

RushPhysicsCore::RushPhysicsCore(float stepDt) : impl_(new Impl(stepDt)) {
    impl_->scene.SetContactListener(impl_.get());
    impl_->scene.SetAllowSleep(false);
    impl_->scene.SetIterations(16);
}

RushPhysicsCore::~RushPhysicsCore() = default;

void RushPhysicsCore::SetGravity(const Vec3& gravity) { impl_->scene.SetGravity(ToRpc(gravity)); }

void RushPhysicsCore::Step(float dt) {
    for (const Impl::PendingForce& pf : impl_->pendingForces) {
        // Handles were validated in ApplyForce; drop if destroyed mid-frame.
        const auto it = impl_->bodies.find(pf.body);
        if (it == impl_->bodies.end() || !it->second) continue;
        it->second->ApplyLinearImpulseAtWorldPoint(ToRpc(pf.force * dt), ToRpc(pf.point));
    }
    impl_->pendingForces.clear();
    (void)dt;  // rpcScene advances by the dt fixed at construction (1/240).
    impl_->scene.Step();
    // Async: publish contacts to CollisionBus; consumers Drain on their stage.
    for (const ContactEvent& e : impl_->contacts) CollisionBus().Publish(e);
    impl_->contacts.clear();
}

rush::Result<BodyHandle> RushPhysicsCore::CreateBody(const BodyDesc& desc) {
    EX_TRY_CHECK(ValidateBodyDesc(desc));
    rpcBodyDef def;
    def.position = ToRpc(desc.transform.position);
    // rpcBodyDef takes axis+angle; convert from quaternion.
    const Quat& q = desc.transform.rotation;
    const float sinHalf = std::sqrt(std::max(0.f, 1.f - q.w * q.w));
    if (sinHalf > 1e-5f) {
        def.axis = rpcVec3{q.x / sinHalf, q.y / sinHalf, q.z / sinHalf};
        def.angle = 2.f * std::atan2(sinHalf, q.w);
    }
    def.bodyType = desc.mass > 0.f ? eDynamicBody : eStaticBody;
    def.linearDamping = desc.linearDamping;
    def.angularDamping = desc.angularDamping;
    def.allowSleep = desc.allowSleep;
    def.userData = desc.userData;

    rpcBody* body = impl_->scene.CreateBody(def);
    if (!body) {
        EX_FAIL("rpc: scene.CreateBody returned null");
    }

    rpcBox* firstBox = nullptr;
    if (!desc.shapes.empty()) {
        // Multi-shape composite: density = mass / sum(volumes) keeps the
        // total mass exact; every shape carries its own part tag.
        float volume = 0.f;
        for (const ShapeDesc& s : desc.shapes) {
            volume += 8.f * s.halfExtents.x * s.halfExtents.y * s.halfExtents.z;
        }
        const float density = desc.mass > 0.f ? desc.mass / std::max(0.001f, volume) : 0.f;
        for (const ShapeDesc& s : desc.shapes) {
            rpcBoxDef boxDef;
            boxDef.Set(ToRpcTransform(s.localTransform), ToRpc(s.halfExtents * 2.f));
            boxDef.SetFriction(s.friction > 0.f ? s.friction : desc.friction);
            boxDef.SetRestitution(desc.restitution);
            if (desc.mass > 0.f) boxDef.SetDensity(density);
            const rpcBox* box = body->AddBox(boxDef);
            if (!box) {
                EX_FAIL("rpc: AddBox returned null");
            }
            // partTag + 1: CarPart 0 would otherwise be a nullptr tag.
            box->SetUserdata(s.partTag >= 0
                                 ? reinterpret_cast<void*>(static_cast<uintptr_t>(s.partTag) + 1u)
                                 : nullptr);
            impl_->boxToBody[box] = impl_->nextHandle;
            if (!firstBox) firstBox = const_cast<rpcBox*>(box);
        }
    } else {
        rpcBoxDef boxDef;
        rpcTransform local;
        rpcIdentity(local);
        // rpcBoxDef::Set takes FULL extents and halves them internally (rpcBox.inl:
        // m_e = extents * 0.5). Passing half-extents would shrink every collider
        // by 2x (and mass by 8x).
        boxDef.Set(local, ToRpc(desc.halfExtents * 2.f));
        boxDef.SetFriction(desc.friction);
        boxDef.SetRestitution(desc.restitution);
        if (desc.mass > 0.f) {
            boxDef.SetDensity(desc.mass / std::max(0.001f, 8.f * desc.halfExtents.x * desc.halfExtents.y *
                                                                desc.halfExtents.z));
        }
        const rpcBox* box = body->AddBox(boxDef);
        if (!box) {
            EX_FAIL("rpc: AddBox returned null");
        }
        impl_->boxToBody[box] = impl_->nextHandle;
        firstBox = const_cast<rpcBox*>(box);
    }
    impl_->bodies[impl_->nextHandle] = body;
    impl_->bodyBoxes[impl_->nextHandle] = firstBox;
    return impl_->nextHandle++;
}

rush::Result<void> RushPhysicsCore::DestroyBody(BodyHandle handle) {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    // Drop box→body reverse lookup for this body's box (if registered).
    if (const auto boxIt = impl_->bodyBoxes.find(handle);
        boxIt != impl_->bodyBoxes.end() && boxIt->second) {
        impl_->boxToBody.erase(boxIt->second);
    }
    impl_->scene.RemoveBody(*body);
    impl_->bodies.erase(handle);
    impl_->bodyBoxes.erase(handle);
    return {};
}

rush::Result<Transform> RushPhysicsCore::GetTransform(BodyHandle handle) const {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    return FromRpcTransform((*body)->GetTransform(), (*body)->GetQuaternion());
}

rush::Result<void> RushPhysicsCore::SetTransform(BodyHandle handle, const Transform& transform) {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    const Quat& q = transform.rotation;
    const float sinHalf = std::sqrt(std::max(0.f, 1.f - q.w * q.w));
    if (sinHalf > 1e-5f) {
        (*body)->SetTransform(ToRpc(transform.position),
                              rpcVec3{q.x / sinHalf, q.y / sinHalf, q.z / sinHalf},
                              2.f * std::atan2(sinHalf, q.w));
    } else {
        (*body)->SetTransform(ToRpc(transform.position));
    }
    return {};
}

rush::Result<Vec3> RushPhysicsCore::GetLinearVelocity(BodyHandle handle) const {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    return FromRpc((*body)->GetLinearVelocity());
}

rush::Result<Vec3> RushPhysicsCore::GetAngularVelocity(BodyHandle handle) const {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    return FromRpc((*body)->GetAngularVelocity());
}

rush::Result<void> RushPhysicsCore::SetLinearVelocity(BodyHandle handle, const Vec3& velocity) {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    (*body)->SetLinearVelocity(ToRpc(velocity));
    return {};
}

rush::Result<void> RushPhysicsCore::SetAngularVelocity(BodyHandle handle, const Vec3& velocity) {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    (*body)->SetAngularVelocity(ToRpc(velocity));
    return {};
}

rush::Result<Vec3> RushPhysicsCore::GetVelocityAtPoint(BodyHandle handle, const Vec3& worldPoint) const {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    return FromRpc((*body)->GetVelocityAtWorldPoint(ToRpc(worldPoint)));
}

rush::Result<void> RushPhysicsCore::ApplyForce(BodyHandle handle, const Vec3& force, const Vec3& worldPoint) {
    // Buffer and convert to an impulse in Step(): qu3e's ApplyForceAtWorldPoint
    // treats its argument as a mass-scaled acceleration (m_force += force *
    // mass) while leaving the torque term unscaled, so passing Newtons makes
    // linear response mass-independent and explodes. Impulses have clean
    // semantics on both axes. Validate the handle now so bad forces fail loud.
    EX_TRY(body, FindBody(impl_->bodies, handle));
    (void)body;
    impl_->pendingForces.push_back({handle, force, worldPoint});
    return {};
}

rush::Result<void> RushPhysicsCore::ApplyImpulse(BodyHandle handle, const Vec3& impulse, const Vec3& worldPoint) {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    (*body)->ApplyLinearImpulseAtWorldPoint(ToRpc(impulse), ToRpc(worldPoint));
    return {};
}

rush::Result<void> RushPhysicsCore::ApplyTorque(BodyHandle handle, const Vec3& torque) {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    (*body)->ApplyTorque(ToRpc(torque));
    return {};
}

RayHit RushPhysicsCore::Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                          BodyHandle ignore) const {
    rpcRaycastData ray;
    // dir is normalized, so t is the arc length to the endpoint.
    ray.Set(ToRpc(origin), ToRpc(Normalize(direction)), maxDistance);

    ClosestHitCallback callback;
    callback.scene = &impl_->scene;
    callback.ray = &ray;
    callback.maxDistance = maxDistance;
    callback.ignore = ignore;
    callback.boxToBody = &impl_->boxToBody;
    impl_->scene.RayCast(&callback, ray);
    return callback.result;
}

std::vector<ContactEvent> RushPhysicsCore::DrainContacts() {
    std::vector<ContactEvent> out;
    CollisionBus().Drain(out);
    return out;
}

// --- Contact friction / raycast: straight native forward into rpc* --------
PhysicsCapability RushPhysicsCore::Capabilities() const {
    return PhysicsCapability::NativeSetFriction | PhysicsCapability::NativeRaycastFriction |
           PhysicsCapability::NativeContactFriction;
}

rush::Result<float> RushPhysicsCore::GetFriction(BodyHandle handle) const {
    EX_TRY(box, FindBox(impl_->bodyBoxes, handle));
    return (*box)->friction;
}

rush::Result<void> RushPhysicsCore::SetFriction(BodyHandle handle, float friction) {
    EX_TRY(box, FindBox(impl_->bodyBoxes, handle));
    if (!std::isfinite(friction)) {
        EX_FAIL("rpc: SetFriction requires finite friction");
    }
    (*box)->friction = friction;
    return {};
}

rush::Result<float> RushPhysicsCore::GetContactFriction(BodyHandle a, BodyHandle b) const {
    EX_TRY(boxA, FindBox(impl_->bodyBoxes, a));
    EX_TRY(boxB, FindBox(impl_->bodyBoxes, b));
    return rpcMixFriction(*boxA, *boxB);
}

}  // namespace apex