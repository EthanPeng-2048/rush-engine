// Pile-up stress: many composite (multi-box) car bodies spawned on top of each
// other at 480 Hz. Reproduces the "several cars collide -> game crashes" report
// and measures the per-step cost of the collision-heavy path.
//
// Not a unit test: it is a harness that exits non-zero on non-finite state or
// on a body that left the world, and prints per-step statistics so the numbers
// can be compared across solver/broadphase changes.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "rpc.h"
#include "dynamics/rpcBody.h"
#include "dynamics/rpcContact.h"
#include "dynamics/rpcContactSolver.h"
#include "dynamics/rpcIsland.h"

namespace {

constexpr int kCarBoxes = 11;  // front wing, rear wing, 2 sidepods, floor,
                               // main chassis + 6 part boxes (see Vehicle::Spawn)
constexpr float kCarHalf[3] = {0.72f, 0.30f, 1.70f};

struct PartBox {
    float px, py, pz;
    float hx, hy, hz;
};

const PartBox kParts[kCarBoxes] = {
    {0.f, 0.06f, 1.68f, 0.72f, 0.05f, 0.18f},   // front wing
    {0.f, 0.62f, -1.66f, 0.70f, 0.12f, 0.15f},  // rear wing
    {-0.70f, 0.20f, 0.f, 0.14f, 0.16f, 1.05f},  // sidepod L
    {0.70f, 0.20f, 0.f, 0.14f, 0.16f, 1.05f},   // sidepod R
    {0.f, -0.02f, 0.5f, 0.55f, 0.06f, 1.00f},   // floor
    {0.f, 0.15f, 0.f, 0.58f, 0.18f, 1.35f},     // main chassis
    {0.f, 0.30f, 0.9f, 0.30f, 0.20f, 0.50f},    // cockpit
    {0.f, 0.35f, -0.5f, 0.35f, 0.25f, 0.60f},   // engine cover
    {-0.35f, 0.10f, -1.0f, 0.20f, 0.15f, 0.35f},// rear left
    {0.35f, 0.10f, -1.0f, 0.20f, 0.15f, 0.35f}, // rear right
    {0.f, 0.45f, -1.30f, 0.25f, 0.10f, 0.30f},  // beam wing
};

rpcBody* SpawnCar(rpcScene& scene, const rpcVec3& position, float yaw,
                  const rpcVec3& velocity = rpcVec3(0.f, 0.f, 0.f)) {
    rpcBodyDef def;
    def.position = position;
    def.axis = rpcVec3(0.f, 1.f, 0.f);
    def.angle = yaw;
    def.bodyType = eDynamicBody;
    def.linearDamping = 0.02f;
    def.angularDamping = 0.25f;
    def.allowSleep = false;
    def.awake = true;
    def.active = true;
    def.linearVelocity = velocity;
    rpcBody* body = scene.CreateBody(def);
    if (!body) {
        std::printf("[FATAL] CreateBody returned null\n");
        std::exit(2);
    }

    float volume = 0.f;
    for (const PartBox& p : kParts) volume += 8.f * p.hx * p.hy * p.hz;
    const float density = 750.f / volume;

    for (int i = 0; i < kCarBoxes; ++i) {
        rpcBoxDef boxDef;
        rpcTransform local;
        rpcIdentity(local);
        local.position = rpcVec3(kParts[i].px, kParts[i].py, kParts[i].pz);
        boxDef.Set(local, rpcVec3(kParts[i].hx * 2.f, kParts[i].hy * 2.f, kParts[i].hz * 2.f));
        boxDef.SetFriction(0.5f);
        boxDef.SetRestitution(0.05f);
        boxDef.SetDensity(density);
        const rpcBox* box = body->AddBox(boxDef);
        if (!box) {
            std::printf("[FATAL] AddBox returned null (scene heap exhausted?)\n");
            std::exit(2);
        }
        box->SetUserdata(reinterpret_cast<void*>(static_cast<uintptr_t>(i) + 1u));
    }
    return body;
}

// Ground made of many static boxes: same order of magnitude as the track so the
// broadphase tree size is representative.
void BuildGround(rpcScene& scene, int halfCount, float cell) {
    for (int x = -halfCount; x < halfCount; ++x) {
        for (int z = -halfCount; z < halfCount; ++z) {
            rpcBodyDef def;
            def.position = rpcVec3((x + 0.5f) * cell, -0.5f, (z + 0.5f) * cell);
            def.bodyType = eStaticBody;
            rpcBody* body = scene.CreateBody(def);
            if (!body) {
                std::printf("[FATAL] ground CreateBody returned null\n");
                std::exit(2);
            }
            rpcBoxDef boxDef;
            rpcTransform local;
            rpcIdentity(local);
            boxDef.Set(local, rpcVec3(cell, 1.f, cell));
            boxDef.SetFriction(0.9f);
            boxDef.SetRestitution(0.0f);
            if (!body->AddBox(boxDef)) {
                std::printf("[FATAL] ground AddBox returned null\n");
                std::exit(2);
            }
        }
    }
}

bool Finite(const rpcVec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    // --info: data-layout report (sizes feed the optimisation doc / cache maths).
    if (argc > 1 && std::strcmp(argv[1], "--info") == 0) {
        std::printf("layout (bytes):\n");
        std::printf("  rpcVec3=%zu  rpcAABB=%zu  rpcMat3=%zu  rpcTransform=%zu  rpcQuaternion=%zu\n",
                    sizeof(rpcVec3), sizeof(rpcAABB), sizeof(rpcMat3), sizeof(rpcTransform),
                    sizeof(rpcQuaternion));
        std::printf("  rpcBox=%zu  rpcBody=%zu\n", sizeof(rpcBox), sizeof(rpcBody));
        std::printf("  rpcContact=%zu  rpcManifold=%zu\n", sizeof(rpcContact), sizeof(rpcManifold));
        std::printf("  rpcContactConstraint=%zu  rpcContactConstraintState=%zu\n",
                    sizeof(rpcContactConstraint), sizeof(rpcContactConstraintState));
        std::printf("  rpcVelocityState=%zu  rpcContactState=%zu\n", sizeof(rpcVelocityState),
                    sizeof(rpcContactState));
        std::printf("  rpcRaycastData=%zu\n", sizeof(rpcRaycastData));
        return 0;
    }

    int cars = 40;
    int seconds = 3;          // simulated seconds
    int hz = 480;
    float spawnSpread = 1.5f;  // cars spawn inside this radius -> instant pile
    bool treeRays = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--cars") == 0 && i + 1 < argc) cars = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) seconds = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--hz") == 0 && i + 1 < argc) hz = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--spread") == 0 && i + 1 < argc) spawnSpread = static_cast<float>(std::atof(argv[++i]));
        else if (std::strcmp(argv[i], "--tree-rays") == 0) treeRays = true;
    }
    if (cars < 1) cars = 1;
    if (hz < 60) hz = 60;

    const float dt = 1.f / static_cast<float>(hz);
    rpcSceneConfig sceneConfig(dt);
    sceneConfig.iterations = 16;
    sceneConfig.rayStrategy = treeRays ? rpcRayStrategy::Tree : rpcRayStrategy::Indexed;
    rpcScene scene(sceneConfig);
    scene.SetAllowSleep(false);
    scene.SetIterations(16);
    std::printf("ray strategy: %s\n", treeRays ? "tree" : "indexed");

    BuildGround(scene, 20, 8.f);  // 40x40 x 8 m = 320 m square, 1600 static boxes

    std::vector<rpcBody*> bodies;
    bodies.reserve(static_cast<size_t>(cars));
    const bool crashMode = spawnSpread <= 0.f;
    for (int i = 0; i < cars; ++i) {
        rpcVec3 p;
        rpcVec3 v(0.f, 0.f, 0.f);
        float yaw = 0.f;
        if (crashMode) {
            // Two columns driving into each other at 60 m/s: head-on car-vs-car
            // impacts, the case players hit when the pack piles into a corner.
            const int row = i / 2;
            const bool south = (i % 2) == 0;
            const float z = south ? 40.f + row * 6.f : -(40.f + row * 6.f);
            p = rpcVec3((i % 4 < 2 ? -9.f : 9.f), 0.55f, z);
            v = rpcVec3(0.f, 0.f, south ? -60.f : 60.f);
            yaw = south ? 3.14159265f : 0.f;
        } else {
            // Ring layout with heavy overlap: every car starts inside the pile.
            const float a = static_cast<float>(i) * 2.39996323f;  // golden angle
            const float r = spawnSpread * std::sqrt(static_cast<float>(i) / static_cast<float>(cars));
            p = rpcVec3(std::cos(a) * r, 0.6f + 0.05f * (i % 4), std::sin(a) * r);
            yaw = a;
        }
        bodies.push_back(SpawnCar(scene, p, yaw, v));
    }
    std::printf("pileup: cars=%d hz=%d seconds=%d spread=%.2f boxes/car=%d mode=%s\n", cars, hz,
                seconds, spawnSpread, kCarBoxes, crashMode ? "head-on" : "overlap");

    const int steps = seconds * hz;
    scene.SetStatsEnabled(true);
    scene.SetDeepStats(true);   // this harness wants point/awake counters
    double worstStepMs = 0.0;
    double totalStepMs = 0.0;
    int badSteps = 0;
    int worstContacts = 0;
    int worstIslands = 0;
    int worstPoints = 0;
    unsigned long long worstNodes = 0;
    unsigned long long worstLeaves = 0;
    rpcScene::Stats worstStats{};
    const int reportEvery = hz / 4 > 0 ? hz / 4 : 1;

    for (int s = 0; s < steps; ++s) {
        const rpcDynamicAABBTree& treeBefore = scene.BroadphaseTree();
        const unsigned long long nodesBefore = treeBefore.QueryNodeVisits();
        const unsigned long long leavesBefore = treeBefore.QueryLeafVisits();
        const auto t0 = std::chrono::steady_clock::now();
        scene.Step();
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();
        const unsigned long long nodes = scene.BroadphaseTree().QueryNodeVisits() - nodesBefore;
        const unsigned long long leaves = scene.BroadphaseTree().QueryLeafVisits() - leavesBefore;
        totalStepMs += ms;
        const rpcScene::Stats& st = scene.GetStats();
        if (ms > worstStepMs) {
            worstStepMs = ms;
            worstStats = st;
            worstNodes = nodes;
            worstLeaves = leaves;
        }
        if (st.contacts > worstContacts) worstContacts = st.contacts;
        if (st.islands > worstIslands) worstIslands = st.islands;
        if (st.contactPoints > worstPoints) worstPoints = st.contactPoints;
        if (s % reportEvery == 0) {
            double fastest = 0.0;
            double fastestW = 0.0;
            for (rpcBody* body : bodies) {
                const rpcVec3 v = body->GetLinearVelocity();
                const rpcVec3 w = body->GetAngularVelocity();
                const double sp = std::sqrt(static_cast<double>(v.x) * v.x +
                                            static_cast<double>(v.y) * v.y +
                                            static_cast<double>(v.z) * v.z);
                const double sw = std::sqrt(static_cast<double>(w.x) * w.x +
                                            static_cast<double>(w.y) * w.y +
                                            static_cast<double>(w.z) * w.z);
                if (sp > fastest) fastest = sp;
                if (sw > fastestW) fastestW = sw;
            }
            std::printf("  step %4d: %8.3f ms  contacts=%5d points=%5d islands=%3d  "
                        "tree nodes=%6llu leaves=%6llu  fast=%.1f m/s w=%.1f rad/s  (narrow=%.3f build=%.3f solve=%.3f sync=%.3f)\n",
                        s, ms, st.contacts, st.contactPoints, st.islands, nodes, leaves, fastest,
                        fastestW, st.narrowphaseMs, st.islandBuildMs, st.islandSolveMs, st.syncMs);
        }

        int nonFinite = 0;
        for (rpcBody* body : bodies) {
            if (!Finite(body->GetTransform().position) || !Finite(body->GetLinearVelocity()) ||
                !Finite(body->GetAngularVelocity())) {
                ++nonFinite;
            }
        }
        if (nonFinite > 0) {
            ++badSteps;
            if (badSteps <= 5) {
                std::printf("[BAD] step %d: %d non-finite cars (first pos %.3f %.3f %.3f)\n", s,
                            nonFinite, bodies[0]->GetTransform().position.x,
                            bodies[0]->GetTransform().position.y,
                            bodies[0]->GetTransform().position.z);
            }
        }
    }

    double escapees = 0.0;
    int far = 0;
    for (rpcBody* body : bodies) {
        const rpcVec3 p = body->GetTransform().position;
        const double d = std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.z) * p.z);
        if (d > 400.0) ++far;
        if (d > escapees) escapees = d;
    }

    std::printf("pileup result: mean=%.3f ms/step worst=%.1f ms  badSteps=%d  maxRadius=%.1f  escaped=%d\n",
                totalStepMs / steps, worstStepMs, badSteps, escapees, far);
    std::printf("  peak: contacts=%d points=%d islands=%d  worst-step breakdown: "
                "narrow=%.1f build=%.1f solve=%.1f sync=%.1f ms  tree nodes=%llu leaves=%llu\n",
                worstContacts, worstPoints, worstIslands, worstStats.narrowphaseMs,
                worstStats.islandBuildMs, worstStats.islandSolveMs, worstStats.syncMs,
                worstNodes, worstLeaves);
    std::printf("  island pools: states=%zu  bytes=%.1f MB (for %d dynamic bodies)\n",
                worstStats.poolContactStates,
                static_cast<double>(worstStats.poolBytes) / (1024.0 * 1024.0),
                scene.GetStats().awakeBodies);
    std::printf("  tree: leaves=%d height=%d\n", scene.BroadphaseTree().Count(),
                scene.BroadphaseTree().Height());
    return badSteps == 0 && far == 0 ? 0 : 1;
}
