// Ray-index conformance test.
//
// The scene can answer ray queries through two interchangeable strategies
// (broadphase BVH, or static grid + dynamic sweep) and through an injected
// factory. This test pins all of them to a brute-force ground truth over the
// same scene, so a strategy swap can never change gameplay-visible results:
//
//   1. broadphase BVH   (rpcRayStrategy::Tree)
//   2. static grid + dynamic sweep (rpcRayStrategy::Indexed)
//   3. injected mock factory (proves the interface is the only coupling)
//
// Ground truth = linear scan over every shape: exact box/ray test, keep nearest.
// Failure conditions: different hit/miss, different shape, or different toi.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "rpc.h"
#include "broadphase/rpcStaticGrid.h"
#include "broadphase/rpcDynamicSweep.h"
#include "dynamics/rpcBody.h"
#include "collision/rpcBox.h"

namespace {

struct Hit {
    bool hit = false;
    float toi = 0.f;
    void* id = nullptr;   // logical shape id (pointer identity is per-scene)
};

// Closest-hit collector: mirrors what the engine adapter does (report by
// shrinking the ray so farther candidates are pruned).
class ClosestHit final : public rpcQueryCallback {
public:
    explicit ClosestHit(rpcRaycastData* ray) : m_ray(ray) {}

    bool ReportShape(rpcBox* box) override {
        if (m_ray->toi <= 0.f) return true;  // inside-box hits are not trusted
        // Closest hit; exact ties broken by shape id so the result does not
        // depend on the traversal order of the active strategy.
        const bool closer = !result.hit || m_ray->toi < result.toi - 1e-6f;
        const bool tie = result.hit && std::fabs(m_ray->toi - result.toi) <= 1e-6f &&
                         box->uid < resultUid;
        if (closer || tie) {
            result.hit = true;
            result.toi = m_ray->toi;
            result.id = box->GetUserdata();
            resultUid = box->uid;
        }
        return true;
    }

    Hit result;
    i32 resultUid = 0x7fffffff;

private:
    rpcRaycastData* m_ray;
};

struct DynamicBodySourceMock final : rpcDynamicBodySource {
    rpcBody** Bodies() override { return bodies.data(); }
    i32 BodyCount() const override { return static_cast<i32>(bodies.size()); }
    std::vector<rpcBody*> bodies;
};

// Injected factory: records that the scene used it, then returns the built-in
// accelerators for the domains (a real alternate accelerator would plug in here).
class CountingRayIndex final : public rpcRayIndex {
public:
    explicit CountingRayIndex(rpcRayIndex* inner) : m_inner(inner) {}
    ~CountingRayIndex() override { delete m_inner; }

    const char* Name() const override { return m_inner->Name(); }
    void Add(rpcBox* box, const rpcAABB& aabb) override { m_inner->Add(box, aabb); }
    void Remove(rpcBox* box) override { m_inner->Remove(box); }
    void Update(rpcBox* box, const rpcAABB& aabb) override { m_inner->Update(box, aabb); }
    void Clear() override { m_inner->Clear(); }
    void Raycast(rpcQueryCallback* cb, rpcRaycastData& ray) const override {
        ++raycastCount;
        m_inner->Raycast(cb, ray);
    }
    void RaycastBatch(rpcQueryCallback* cb, rpcRaycastData* rays, i32 count) const override {
        raycastBatchCount += count;
        m_inner->RaycastBatch(cb, rays, count);
    }

    mutable int raycastCount = 0;
    mutable int raycastBatchCount = 0;

private:
    rpcRayIndex* m_inner;
};

struct Factory final : rpcRayIndexFactory {
    rpcRayIndex* CreateStaticIndex(r32 cellSize) override {
        staticCount++;
        return new CountingRayIndex(new rpcStaticGrid(cellSize));
    }

    rpcRayIndex* CreateDynamicIndex(rpcDynamicBodySource* source) override {
        dynamicCount++;
        return new CountingRayIndex(new rpcDynamicSweep(source));
    }

    int staticCount = 0;
    int dynamicCount = 0;
};

rpcBox* AddSlab(rpcBody* body, const rpcVec3& center, const rpcVec3& half, const rpcQuaternion& q) {
    rpcBoxDef def;
    rpcTransform local;
    rpcIdentity(local);
    local.position = center;
    def.Set(local, rpcVec3(half.x * 2.f, half.y * 2.f, half.z * 2.f));
    const rpcBox* box = body->AddBox(def);
    (void)q;
    return const_cast<rpcBox*>(box);
}

rpcBody* CreateBoxBody(rpcScene& scene, const rpcVec3& position, const rpcQuaternion& rotation,
                       float half, bool dynamic, int& nextId) {
    rpcBodyDef def;
    def.position = position;
    rpcVec3 axis(0.f, 1.f, 0.f);
    float angle = 0.f;
    rotation.ToAxisAngle(&axis, &angle);
    def.axis = axis;
    def.angle = angle;
    def.bodyType = dynamic ? eDynamicBody : eStaticBody;
    def.allowSleep = false;
    rpcBody* body = scene.CreateBody(def);
    if (!body) {
        std::printf("[FATAL] CreateBody returned null\n");
        std::exit(2);
    }
    rpcBoxDef boxDef;
    rpcTransform local;
    rpcIdentity(local);
    boxDef.Set(local, rpcVec3(half * 2.f, half * 0.35f, half * 2.f));
    boxDef.SetDensity(dynamic ? 500.f : 0.f);
    const rpcBox* created = body->AddBox(boxDef);
    if (!created) {
        std::printf("[FATAL] AddBox returned null\n");
        std::exit(2);
    }
    created->SetUserdata(reinterpret_cast<void*>(static_cast<uintptr_t>(++nextId)));
    return body;
}

// Ground truth: linear scan, exact test, closest hit.
Hit BruteForce(const std::vector<rpcBody*>& bodies, const rpcVec3& origin, const rpcVec3& dir,
               float maxDist) {
    rpcRaycastData ray;
    ray.Set(origin, dir, maxDist);

    Hit best;
    i32 bestUid = 0x7fffffff;
    for (rpcBody* body : bodies) {
        const rpcTransform tx = body->GetTransform();
        for (rpcBox* box = body->GetBoxes(); box; box = box->next) {
            rpcRaycastData probe = ray;
            if (!box->Raycast(tx, &probe)) continue;
            if (probe.toi <= 0.f) continue;
            const bool closer = !best.hit || probe.toi < best.toi - 1e-6f;
            const bool tie = best.hit && std::fabs(probe.toi - best.toi) <= 1e-6f && box->uid < bestUid;
            if (closer || tie) {
                best.hit = true;
                best.toi = probe.toi;
                best.id = box->GetUserdata();
                bestUid = box->uid;
            }
        }
    }
    return best;
}

Hit QueryScene(const rpcScene& scene, const rpcVec3& origin, const rpcVec3& dir, float maxDist) {
    rpcRaycastData ray;
    ray.Set(origin, dir, maxDist);
    ClosestHit cb(&ray);
    scene.RayCast(&cb, ray);
    return cb.result;
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    int dynamicBodies = 24;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--dynamic") == 0 && i + 1 < argc)
            dynamicBodies = std::atoi(argv[++i]);
    }

    // Scene: a rotated slab floor (like the track), a few static obstacles, and
    // dynamic multi-shape bodies (like the cars).
    rpcSceneConfig treeConfig(1.f / 240.f);
    treeConfig.iterations = 8;
    treeConfig.rayStrategy = rpcRayStrategy::Tree;
    rpcSceneConfig indexedConfig = treeConfig;
    indexedConfig.rayStrategy = rpcRayStrategy::Indexed;

    std::vector<rpcBox*> boxes;
    std::vector<rpcBody*> bodies;

    const auto buildScene = [&](rpcScene& scene) {
        int nextId = 0;
        // Floor: 30 x 30 slabs of 8 m, every other one rotated 30 degrees, so
        // rotated local transforms and multi-cell shapes are both covered.
        for (int x = -15; x < 15; ++x) {
            for (int z = -15; z < 15; ++z) {
                const bool rot = ((x + z) & 1) != 0;
                rpcQuaternion q;
                if (rot) q.Set(rpcVec3(0.f, 1.f, 0.f), 0.5235988f);
                rpcBody* body = CreateBoxBody(scene, rpcVec3(x * 8.f, -0.25f, z * 8.f), q, 4.f, false, nextId);
                bodies.push_back(body);
            }
        }
        // Static walls.
        for (int i = 0; i < 40; ++i) {
            rpcQuaternion q;
            q.Set(rpcVec3(0.f, 1.f, 0.f), 0.2f * static_cast<float>(i));
            bodies.push_back(CreateBoxBody(scene, rpcVec3(-120.f + 6.f * i, 2.f, 40.f), q, 1.5f, false, nextId));
        }
        // Dynamic bodies.
        for (int i = 0; i < dynamicBodies; ++i) {
            rpcQuaternion q;
            q.Set(rpcVec3(0.f, 1.f, 0.f), 0.3f * static_cast<float>(i));
            rpcBody* body = CreateBoxBody(scene, rpcVec3(-30.f + 1.5f * (i % 40), 0.5f, -12.f + 1.1f * (i / 40)), q,
                                          1.2f, true, nextId);
            // Composite: a second, offset shape (the "wing").
            rpcBoxDef wing;
            rpcTransform local;
            rpcIdentity(local);
            local.position = rpcVec3(0.f, 0.4f, 1.6f);
            wing.Set(local, rpcVec3(1.4f, 0.1f, 0.4f));
            wing.SetDensity(500.f);
            const rpcBox* wingBox = body->AddBox(wing);
            wingBox->SetUserdata(reinterpret_cast<void*>(static_cast<uintptr_t>(++nextId)));
            bodies.push_back(body);
        }
    };

    rpcScene treeScene(treeConfig);
    rpcScene indexedScene(indexedConfig);
    buildScene(treeScene);
    buildScene(indexedScene);

    // The mock/injected run: proves the scene talks only to rpcRayIndex.
    rpcSceneConfig injectedConfig = indexedConfig;
    Factory factory;
    injectedConfig.rayFactory = &factory;
    rpcScene injectedScene(injectedConfig);
    {
        // Rebuild the same geometry into the injected scene (separate world).
        buildScene(injectedScene);
    }

    std::printf("ray-index conformance: bodies=%zu\n", bodies.size() / 3);

    // Random rays from above and sideways, plus a few degenerate ones.
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> pos(-60.f, 60.f);
    std::uniform_real_distribution<float> dirDist(-1.f, 1.f);
    std::uniform_real_distribution<float> len(0.5f, 40.f);

    int failures = 0;
    int hits = 0;
    const int kRays = 4000;
    for (int i = 0; i < kRays; ++i) {
        rpcVec3 origin;
        rpcVec3 dir;
        if (i % 4 == 0) {
            origin = rpcVec3(pos(rng), 30.f, pos(rng));  // straight down
            dir = rpcVec3(0.f, -1.f, 0.f);
        } else if (i % 4 == 1) {
            origin = rpcVec3(pos(rng), 0.8f, pos(rng));  // shallow, like a suspension ray
            dir = rpcNormalize(rpcVec3(dirDist(rng) * 0.2f, -1.f, dirDist(rng) * 0.2f));
        } else {
            origin = rpcVec3(pos(rng), pos(rng), pos(rng));
            dir = rpcNormalize(rpcVec3(dirDist(rng), dirDist(rng), dirDist(rng)));
        }
        const float maxDist = (i % 4 == 1) ? 0.75f : len(rng);

        const Hit truth = BruteForce(bodies, origin, dir, maxDist);
        const Hit tree = QueryScene(treeScene, origin, dir, maxDist);
        const Hit indexed = QueryScene(indexedScene, origin, dir, maxDist);
        const Hit injected = QueryScene(injectedScene, origin, dir, maxDist);

        if (truth.hit) ++hits;

        const auto same = [](const Hit& a, const Hit& b) {
            if (a.hit != b.hit) return false;
            if (!a.hit) return true;
            return a.id == b.id && std::fabs(a.toi - b.toi) < 1e-3f;
        };

        const bool ok = same(truth, tree) && same(truth, indexed) && same(truth, injected);
        if (!ok) {
            ++failures;
            if (failures <= 8) {
                std::printf("[FAIL] ray %d: truth=%d(%.4f) tree=%d(%.4f) indexed=%d(%.4f) injected=%d(%.4f)\n",
                            i, (int)truth.hit, truth.toi, (int)tree.hit, tree.toi, (int)indexed.hit,
                            indexed.toi, (int)injected.hit, injected.toi);
            }
        }
    }

    std::printf("conformance: rays=%d hits=%d failures=%d\n", kRays, hits, failures);
    std::printf("injection: static=%d dynamic=%d  (factory used)\n", factory.staticCount,
                factory.dynamicCount);

    if (factory.staticCount != 1 || factory.dynamicCount != 1) {
        std::printf("[FAIL] injected factory was not used by the scene\n");
        ++failures;
    }

    // Cost probe: same ray set through both strategies (coarse, single run).
    const auto timeStrategy = [&](rpcScene& scene) {
        const auto t0 = std::chrono::steady_clock::now();
        float sink = 0.f;
        for (int i = 0; i < kRays; ++i) {
            const rpcVec3 origin(pos(rng), 0.8f, pos(rng));
            const rpcVec3 dir = rpcNormalize(rpcVec3(dirDist(rng) * 0.2f, -1.f, dirDist(rng) * 0.2f));
            rpcRaycastData ray;
            ray.Set(origin, dir, 0.75f);
            ClosestHit cb(&ray);
            scene.RayCast(&cb, ray);
            sink += cb.result.toi;
        }
        const double us = std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - t0).count();
        return std::make_pair(us / kRays, sink);
    };
    const auto treeCost = timeStrategy(treeScene);
    const auto indexedCost = timeStrategy(indexedScene);
    std::printf("cost: tree=%.3f us/ray  indexed=%.3f us/ray  (sink %.1f/%.1f)\n", treeCost.first,
                indexedCost.first, treeCost.second, indexedCost.second);

    return failures == 0 ? 0 : 1;
}
