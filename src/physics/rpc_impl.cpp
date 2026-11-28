module;
#include <algorithm>
#include <chrono>
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
#include "common/rpcWorkerPool.h"
#include "raycast_probe.h"

module rush.engine.physics;
import rush.engine.core;

namespace apex {

namespace {

// Milliseconds between two steady-clock samples (diagnostics only).
template <typename TimePoint>
double secs(TimePoint from, TimePoint to) {
    return std::chrono::duration<double, std::milli>(to - from).count();
}

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
// The handle table is a dense vector indexed by BodyHandle (handles are
// allocated strictly increasing and never recycled), so the hot-path lookup
// is one array read instead of a hash probe.
struct BodyTable {
    std::vector<rpcBody*> bodies;   // index = handle, null = freed slot
    BodyHandle next = 1;

    rpcBody* Get(BodyHandle handle) const {
        return handle < bodies.size() ? bodies[handle] : nullptr;
    }
    void Set(BodyHandle handle, rpcBody* body) {
        if (handle >= bodies.size()) bodies.resize(handle + 1, nullptr);
        bodies[handle] = body;
    }
    void Erase(BodyHandle handle) {
        if (handle < bodies.size()) bodies[handle] = nullptr;
    }
};

// Hot-path lookup: the vehicle calls these per substep with handles it owns
// for the whole race, so Release skips the Result wrapping entirely (a bad
// handle is a programming error caught by the Debug assert). The checked
// EX_FAIL channel stays on the cold paths (CreateBody/DestroyBody/Friction).
static rpcBody* FindBodyFast(const BodyTable& table, BodyHandle handle) {
    rpcBody* body = table.Get(handle);
    assert(body);
    return body;
}

rush::Result<rpcBody*> FindBody(const BodyTable& table, BodyHandle handle) {
    rpcBody* body = table.Get(handle);
    if (!body) {
        EX_FAIL("rpc: invalid body handle " + std::to_string(handle));
    }
    return body;
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

// Sums buffered per-body impulses in O(1) per force. The previous version
// scanned the already-aggregated list for every buffered force (O(bodies) per
// force): ~11 forces per car made that ~0.13 ms per substep at 120 cars — a
// fifth of the whole physics step, spent comparing pointers over a 24 B
// struct. Handles are dense and never recycled, so the handle *is* the slot.
class ImpulseAccumulator {
public:
    void Begin() {
        if (++epoch_ == 0) {  // stamp wrap: start the table over
            for (Slot& s : slots_) s.stamp = 0;
            epoch_ = 1;
        }
        touched_.clear();
    }

    void Add(BodyHandle handle, const Vec3& linear, const Vec3& angular) {
        if (handle >= slots_.size()) {
            slots_.resize(std::max<size_t>(static_cast<size_t>(handle) + 1, slots_.size() * 2 + 1));
        }
        Slot& slot = slots_[handle];
        if (slot.stamp != epoch_) {
            slot.stamp = epoch_;
            slot.linear = {};
            slot.angular = {};
            touched_.push_back(handle);
        }
        slot.linear += linear;
        slot.angular += angular;
    }

    const std::vector<BodyHandle>& Touched() const { return touched_; }
    const Vec3& Linear(BodyHandle handle) const { return slots_[handle].linear; }
    const Vec3& Angular(BodyHandle handle) const { return slots_[handle].angular; }

private:
    struct Slot {
        Vec3 linear;
        Vec3 angular;
        unsigned stamp = 0;
    };
    std::vector<Slot> slots_;
    std::vector<BodyHandle> touched_;
    unsigned epoch_ = 0;
};

// Captures contacts produced during the most recent scene step.
struct RushPhysicsCore::Impl final : public rpcContactListener {
public:
    struct PendingForce {
        BodyHandle handle = 0;
        Vec3 force;
        Vec3 point;
    };

    explicit Impl(float stepDt)
        // rpcScene advances by the fixed dt given here on every Step(). The
        // game calls Step() once per physics substep, so use the same rate.
        : scene(MakeConfig(stepDt)) {
        // Pre-warm the process-wide worker pool at world creation (race
        // load), so enabling island parallelism later never pays thread
        // creation on the physics hot path.
        (void)rpcWorkerPool::Shared();
    }

    // Scheduling policy lives in the config (see rpcSceneConfig): the new
    // contact budget is what turns "a pile-up exposes 12 000 pairs in one step"
    // from a 20 ms frame into a few bounded steps.
    static rpcSceneConfig MakeConfig(float stepDt) {
        rpcSceneConfig config(stepDt);
        config.gravity = rpcVec3{0.f, -9.81f, 0.f};
        config.iterations = 12;
        config.newContactsPerStep = 0;  // unlimited: contacts ARE the anti-penetration work
        // TGS: solve every substep (480 Hz stiffness) but detect once per frame
        // (60 Hz). The game steps 8 substeps per frame at 480 Hz.
        config.contactRefreshSubsteps = 4;
        return config;
    }

    rpcScene scene;
    BodyTable bodies;   // dense handle->body table (index = handle)
    std::unordered_map<BodyHandle, rpcBox*> bodyBoxes;
    BodyHandle nextHandle = 1;
    std::vector<ContactEvent> contacts;
    std::unordered_map<const rpcBox*, BodyHandle> boxToBody;
    std::vector<PendingForce> pendingForces;
    ImpulseAccumulator impulses;

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
    i32 resultUid = 0x7fffffff;  // tie-break id of the current best hit
    // Candidate "the origin started inside this collider" report. Only a
    // candidate: a real hit further along the ray wins, because a hardpoint
    // buried in a barrier still has the road underneath it.
    RayHit insideCandidate;
    bool sawInside = false;
    uint64_t leafTests = 0;  // TEMP probe

    // Final answer: the closest real hit, else the buried report, else a miss.
    RayHit Finish() const {
        if (result.hit) return result;
        if (!sawInside) return result;
        RayHit out = insideCandidate;
        out.hit = false;
        out.inside = true;
        return out;
    }

    bool ReportShape(rpcBox* box) override {
        ++leafTests;  // TEMP probe
        const auto it = boxToBody->find(box);
        if (it != boxToBody->end() && it->second == ignore) return true;

        // `ray->toi` / `ray->normal` were just written by the scene's test.
        const float t = ray->toi;
        // A ray that starts inside a box reports toi=0 with an uninitialized
        // normal (rpcBox::Raycast never writes n0 when all entry t0 < 0), so
        // those hits are untrustworthy. They are not a miss either: remember
        // them as `inside` (see RayHit::inside) so a buried suspension
        // hardpoint stays distinguishable from an airborne one — without ever
        // overwriting a real hit (box order is not deterministic).
        if (t <= 0.f) {
            if (!sawInside && box->TestPoint(box->body->GetTransform(), ray->start)) {
                sawInside = true;
                insideCandidate.body = it != boxToBody->end() ? it->second : kInvalidBody;
                insideCandidate.point = FromRpc(ray->start);
                insideCandidate.userData = box->GetUserdata();
                insideCandidate.friction = box->friction;
            }
            return true;
        }
        // Closest hit, with exact distance ties broken by shape id: the answer
        // must not depend on which ray strategy (tree / grid+sweep) replied.
        const bool closer = !result.hit || t < ray->t - 1e-6f;
        const bool tie = result.hit && std::fabs(t - ray->t) <= 1e-6f && box->uid < resultUid;
        if (closer || tie) {
            if (closer) ray->t = t;  // prune anything farther than the best hit
            resultUid = box->uid;
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

// Per-ray closest-hit selection for the batch query. Each ray keeps its own
// rpcRaycastData (the batch traversal writes per-ray toi/normal), so the
// closest-hit prune applies to that ray alone.
class BatchClosestHitCallback final : public rpcQueryCallback {
public:
    const rpcScene* scene = nullptr;
    rpcRaycastData* rays = nullptr;
    float maxDistance = 0.f;
    BodyHandle ignore = kInvalidBody;
    const std::unordered_map<const rpcBox*, BodyHandle>* boxToBody = nullptr;
    RayHit results[32];
    i32 resultUid[32];
    // Per-ray "origin started inside a collider" candidates. A real hit on the
    // same ray wins over them (see Finish()).
    RayHit insideCandidate[32];
    bool sawInside[32] = {};
    uint64_t leafTests = 0;  // TEMP probe

    RayHit Finish(int rayIndex) const {
        const RayHit& result = results[rayIndex];
        if (result.hit || !sawInside[rayIndex]) return result;
        RayHit out = insideCandidate[rayIndex];
        out.hit = false;
        out.inside = true;
        return out;
    }

    bool ReportShape(rpcBox* box) override {
        // The batch traversal always reports with the per-ray overload.
        (void)box;
        return true;
    }

    bool ReportShape(rpcBox* box, int rayIndex, rpcRaycastData* ray) override {
        ++leafTests;  // TEMP probe
        const auto it = boxToBody->find(box);
        if (it != boxToBody->end() && it->second == ignore) return true;

        // `ray->toi` / `ray->normal` were just written by the scene's test.
        const float t = ray->toi;
        // A ray that starts inside a box reports toi=0 with an uninitialized
        // normal (rpcBox::Raycast never writes n0 when all entry t0 < 0), so
        // those hits are untrustworthy. They are not a miss either: remember
        // them as `inside` (see RayHit::inside) so a buried suspension
        // hardpoint stays distinguishable from an airborne one — without ever
        // overwriting a real hit (box order is not deterministic).
        if (t <= 0.f) {
            if (!sawInside[rayIndex] &&
                box->TestPoint(box->body->GetTransform(), ray->start)) {
                sawInside[rayIndex] = true;
                RayHit& cand = insideCandidate[rayIndex];
                cand.body = it != boxToBody->end() ? it->second : kInvalidBody;
                cand.point = FromRpc(ray->start);
                cand.userData = box->GetUserdata();
                cand.friction = box->friction;
            }
            return true;
        }
        RayHit& result = results[rayIndex];
        // Closest hit per ray, ties broken by shape id (strategy-independent).
        const bool closer = !result.hit || t < ray->t - 1e-6f;
        const bool tie = result.hit && std::fabs(t - ray->t) <= 1e-6f && box->uid < resultUid[rayIndex];
        if (closer || tie) {
            if (closer) ray->t = t;  // prune anything farther than the best hit
            resultUid[rayIndex] = box->uid;
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
    // Solver budget scales with the substep rate: 16 iterations at 240 Hz, half
    // that at 480 Hz, capped at 16 for slower rates (the smaller dt already
    // improves convergence, so paying 32+ iterations at 120 Hz buys nothing).
    // The cost per simulated second stays roughly constant while the
    // per-substep cost — the one multiplied by the substep count — shrinks at
    // high rates.
    const float iters = 16.f * (stepDt * 240.f);
    impl_->scene.SetIterations(std::clamp(static_cast<int>(std::lround(iters)), 4, 16));
}

RushPhysicsCore::~RushPhysicsCore() = default;

void RushPhysicsCore::SetGravity(const Vec3& gravity) { impl_->scene.SetGravity(ToRpc(gravity)); }

void RushPhysicsCore::Step(float dt) {
    using Clock = std::chrono::steady_clock;
    const Clock::time_point wrapStart = diag_ ? Clock::now() : Clock::time_point{};
    // Aggregate the buffered forces into one impulse application per body:
    // linear impulses sum directly, world angular impulses sum as well
    // (r x J), so each body pays one inertia application instead of ~11.
    // Mathematically identical; only the float summation order differs.
    impl_->impulses.Begin();
    for (const Impl::PendingForce& pf : impl_->pendingForces) {
        // Handle was validated in ApplyForce; drop if destroyed mid-frame.
        rpcBody* body = impl_->bodies.Get(pf.handle);
        if (!body) continue;
        const Vec3 j = pf.force * dt;
        const Vec3 r = pf.point - FromRpc(body->GetWorldCenter());
        impl_->impulses.Add(pf.handle, j, Cross(r, j));
    }
    for (const BodyHandle handle : impl_->impulses.Touched()) {
        rpcBody* body = impl_->bodies.Get(handle);
        body->ApplyLinearImpulse(ToRpc(impl_->impulses.Linear(handle)));
        body->ApplyAngularImpulse(ToRpc(impl_->impulses.Angular(handle)));
    }
    impl_->pendingForces.clear();
    (void)dt;  // rpcScene advances by the dt fixed at construction (1/240).
    const Clock::time_point sceneStart = diag_ ? Clock::now() : Clock::time_point{};
    impl_->scene.Step();
    const Clock::time_point sceneEnd = diag_ ? Clock::now() : Clock::time_point{};

    // Accumulate stage timings into means (the last substep alone is often
    // empty, so a single-step report would read as zeros). Zero-cost when
    // diagnostics are off: every field stays 0 and nothing is added.
    const rpcScene::Stats& st = impl_->scene.GetStats();
    if (diag_) {
        stageBroadMs_ += st.broadphaseMs;
        stageProxyMs_ += st.proxyMs;
        stagePairMs_ += st.pairMs;
        stageNarrowMs_ += st.narrowphaseMs;
        stageBuildMs_ += st.islandBuildMs;
        stageSolveMs_ += st.islandSolveMs;
        stageForceMs_ += st.forceMs;
        stageStepMs_ += st.stepMs;
        stageSyncMs_ += st.syncMs;
        if (st.newContacts > stageNewContactsPeak_) stageNewContactsPeak_ = st.newContacts;
        if (st.deferredPairs > stageDeferredPeak_) stageDeferredPeak_ = st.deferredPairs;
        if (st.movedProxies > stageMovedPeak_) stageMovedPeak_ = st.movedProxies;
        if (st.pairsProduced > stagePairsProducedPeak_) stagePairsProducedPeak_ = st.pairsProduced;
        if (st.newContacts > stagePeak_.newContacts) stagePeak_.newContacts = st.newContacts;
        // Worst-case step: the number the player experiences as a hitch.
        stagePeak_.stepMs = std::max(stagePeak_.stepMs, st.stepMs);
        stagePeak_.proxyMs = std::max(stagePeak_.proxyMs, st.proxyMs);
        stagePeak_.pairMs = std::max(stagePeak_.pairMs, st.pairMs);
        stagePeak_.narrowphaseMs = std::max(stagePeak_.narrowphaseMs, st.narrowphaseMs);
        stagePeak_.islandBuildMs = std::max(stagePeak_.islandBuildMs, st.islandBuildMs);
        stagePeak_.solveMs = std::max(stagePeak_.solveMs, st.islandSolveMs);
        stagePeak_.forceMs = std::max(stagePeak_.forceMs, st.forceMs);
        stagePeak_.syncMs = std::max(stagePeak_.syncMs, st.syncMs);
        if (st.deferredPairs > stagePeak_.deferredPairs) stagePeak_.deferredPairs = st.deferredPairs;
        // (worst-step pair sub-timers were dropped with the sort)
        if (st.contacts > stagePeak_.contacts) stagePeak_.contacts = st.contacts;
        if (st.pairsProduced > stagePeak_.pairsProduced) stagePeak_.pairsProduced = st.pairsProduced;
        if (st.movedProxies > stagePeak_.movedProxies) stagePeak_.movedProxies = st.movedProxies;
        ++stageSteps_;
        if (st.contacts > stageContactsPeak_) stageContactsPeak_ = st.contacts;
        if (st.contactPoints > stagePointsPeak_) stagePointsPeak_ = st.contactPoints;
        if (st.islands > stageIslandsPeak_) stageIslandsPeak_ = st.islands;
    }
    // Async: publish contacts to CollisionBus; consumers Drain on their stage.
    for (const ContactEvent& e : impl_->contacts) CollisionBus().Publish(e);
    impl_->contacts.clear();
    if (diag_) {
        const Clock::time_point wrapEnd = Clock::now();
        stageWrapMs_ += secs(wrapStart, sceneStart) + secs(sceneEnd, wrapEnd);
    }
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
    impl_->bodies.Set(impl_->nextHandle, body);
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
    // Buffered forces are keyed by handle, and the handle table entry is
    // cleared below, so Step() skips this body's forces without a purge pass.
    impl_->scene.RemoveBody(*body);
    impl_->bodies.Erase(handle);
    impl_->bodyBoxes.erase(handle);
    return {};
}

rush::Result<Transform> RushPhysicsCore::GetTransform(BodyHandle handle) const {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    return FromRpcTransform((*body)->GetTransform(), (*body)->GetQuaternion());
}

rush::Result<void> RushPhysicsCore::SetTransform(BodyHandle handle, const Transform& transform) {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    // GetTransform() hands back the body ORIGIN, SetTransform() wants the mass
    // center (see the qu3e adapter) — fold the offset back in or every set
    // shifts a dynamic body by R * localCenter.
    const Vec3 center =
        transform.position + Rotate(transform.rotation, FromRpc((*body)->GetLocalCenter()));
    const Quat& q = transform.rotation;
    const float sinHalf = std::sqrt(std::max(0.f, 1.f - q.w * q.w));
    if (sinHalf > 1e-5f) {
        (*body)->SetTransform(ToRpc(center), rpcVec3{q.x / sinHalf, q.y / sinHalf, q.z / sinHalf},
                              2.f * std::atan2(sinHalf, q.w));
    } else {
        (*body)->SetTransform(ToRpc(center));
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
    // Hot path (suspension, 4x per substep): unchecked fast lookup.
    rpcBody* body = FindBodyFast(impl_->bodies, handle);
    return FromRpc(body->GetVelocityAtWorldPoint(ToRpc(worldPoint)));
}

rush::Result<ChassisSnapshot> RushPhysicsCore::FetchChassis(BodyHandle handle) const {
    // Hot path (once per substep per vehicle): unchecked fast lookup.
    rpcBody* body = FindBodyFast(impl_->bodies, handle);
    ChassisSnapshot out;
    out.transform = FromRpcTransform(body->GetTransform(), body->GetQuaternion());
    out.linearVelocity = FromRpc(body->GetLinearVelocity());
    out.angularVelocity = FromRpc(body->GetAngularVelocity());
    return out;
}

rush::Result<void> RushPhysicsCore::ApplyForce(BodyHandle handle, const Vec3& force, const Vec3& worldPoint) {
    // Buffer and convert to an impulse in Step(): qu3e's ApplyForceAtWorldPoint
    // treats its argument as a mass-scaled acceleration (m_force += force *
    // mass) while leaving the torque term unscaled, so passing Newtons makes
    // linear response mass-independent and explodes. Impulses have clean
    // semantics on both axes. Hot path (~11x per substep per car): buffer the
    // handle only — resolving the body here would be a lookup Step() has to
    // repeat anyway to aggregate by body.
    impl_->pendingForces.push_back({handle, force, worldPoint});
    return {};
}

rush::Result<void> RushPhysicsCore::ApplyImpulse(BodyHandle handle, const Vec3& impulse, const Vec3& worldPoint) {
    EX_TRY(body, FindBody(impl_->bodies, handle));
    (*body)->ApplyLinearImpulseAtWorldPoint(ToRpc(impulse), ToRpc(worldPoint));
    return {};
}

rush::Result<void> RushPhysicsCore::ApplyTorque(BodyHandle handle, const Vec3& torque) {
    // Hot path (aero/ESC, once or twice per substep): unchecked fast lookup.
    rpcBody* body = FindBodyFast(impl_->bodies, handle);
    body->ApplyTorque(ToRpc(torque));
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
    const auto t0 = std::chrono::steady_clock::now();  // TEMP probe
    impl_->scene.RayCast(&callback, ray, ignore == kInvalidBody ? nullptr : impl_->bodies.Get(ignore));
    RpcRaycastProbeAdd(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count(),
                       callback.leafTests);
    return callback.Finish();
}

void RushPhysicsCore::RaycastBatch(const Vec3* origins, const Vec3& direction, float maxDistance,
                                   int count, BodyHandle ignore, RayHit* out) const {
    if (count <= 0) return;
    assert(count <= 32);  // broadphase per-batch mask contract
    // dir is normalized once — identical to `count` separate Raycast calls.
    const rpcVec3 dir = ToRpc(Normalize(direction));
    rpcRaycastData rays[32];
    for (int j = 0; j < count; ++j) rays[j].Set(ToRpc(origins[j]), dir, maxDistance);

    BatchClosestHitCallback callback;
    callback.rays = rays;
    callback.maxDistance = maxDistance;
    callback.ignore = ignore;
    callback.boxToBody = &impl_->boxToBody;
    const auto t0 = std::chrono::steady_clock::now();  // TEMP probe
    const u64 nodesBefore = impl_->scene.BroadphaseTree( ).QueryNodeVisits( );
    impl_->scene.RayCastBatch(&callback, rays, count,
                             ignore == kInvalidBody ? nullptr : impl_->bodies.Get(ignore));
    const double nodes = static_cast<double>( impl_->scene.BroadphaseTree( ).QueryNodeVisits( ) - nodesBefore );
    // Probe reports per-ray-equivalent numbers (amortized over count).
    RpcRaycastProbeAdd(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / count,
                       static_cast<double>(callback.leafTests) / count, nodes / count);
    for (int j = 0; j < count; ++j) out[j] = callback.Finish(j);
}

std::vector<ContactEvent> RushPhysicsCore::DrainContacts() {
    std::vector<ContactEvent> out;
    CollisionBus().Drain(out);
    return out;
}

void RushPhysicsCore::SetParallelIslands(bool enabled) {
    impl_->scene.SetParallelIslands(enabled);
}

void RushPhysicsCore::SetDiagnostics(bool on) {
    // Only when a consumer reads the report do we pay for the clock reads.
    diag_ = on;
    impl_->scene.SetStatsEnabled(on);
}

void RushPhysicsCore::ResetStageStats() {
    stageBroadMs_ = stageProxyMs_ = stagePairMs_ = stageSyncMs_ = 0.0;
    stageNarrowMs_ = stageBuildMs_ = stageSolveMs_ = stageForceMs_ = 0.0;
    stageStepMs_ = stageWrapMs_ = 0.0;
    stagePairQueryMs_ = 0.0;
    stageMovedPeak_ = stagePairsProducedPeak_ = stageNewContactsPeak_ = stageDeferredPeak_ = 0;
    stagePeak_ = IPhysicsWorld::StageStats::Peak{};
    stageSteps_ = 0;
    stageContactsPeak_ = stagePointsPeak_ = stageIslandsPeak_ = 0;
}

void RushPhysicsCore::ReadStageStats(StageStats& out) const {
    const rpcScene::Stats& s = impl_->scene.GetStats();
    out.broadphaseMs = s.broadphaseMs;
    out.narrowphaseMs = s.narrowphaseMs;
    out.islandBuildMs = s.islandBuildMs;
    out.solveMs = s.islandSolveMs;
    out.contacts = s.contacts;
    out.contactPoints = s.contactPoints;
    out.islands = s.islands;
    out.syncMs = s.syncMs;
    out.proxyMs = s.proxyMs;
    out.pairMs = s.pairMs;
    out.gridCells = impl_->scene.StaticGridCells();

    // Report means over the whole run: a quiet final substep would otherwise
    // read as zeros, and a single step is too noisy to tune against.
    if (stageSteps_ > 0) {
        const double steps = static_cast<double>(stageSteps_);
        out.broadphaseMs = stageBroadMs_ / steps;
        out.proxyMs = stageProxyMs_ / steps;
        out.pairMs = stagePairMs_ / steps;
        out.forceMs = stageForceMs_ / steps;
        out.stepMs = stageStepMs_ / steps;
        out.wrapMs = stageWrapMs_ / steps;
        out.syncMs = stageSyncMs_ / steps;
        out.newContacts = stageNewContactsPeak_;
        out.deferredPairs = stageDeferredPeak_;
        out.movedProxies = stageMovedPeak_;
        out.pairsProduced = stagePairsProducedPeak_;
        // (pair sub-timers removed: the dedupe is now O(1), no sort stage)
        out.peak = stagePeak_;
        out.narrowphaseMs = stageNarrowMs_ / steps;
        out.islandBuildMs = stageBuildMs_ / steps;
        out.solveMs = stageSolveMs_ / steps;
        out.contacts = stageContactsPeak_;
        out.contactPoints = stagePointsPeak_;
        out.islands = stageIslandsPeak_;
        out.steps = stageSteps_;
    }
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
