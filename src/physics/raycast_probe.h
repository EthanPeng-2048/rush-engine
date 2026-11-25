// TEMPORARY diagnostic probe — removed before the perf commit lands.
// Counts IPhysicsWorld::Raycast cost per backend (single-threaded bench only).
#pragma once

#include <cstdint>

namespace apex {

struct RaycastProbeStats {
    uint64_t calls = 0;
    double micros = 0.0;
    uint64_t leafTests = 0;
    uint64_t nodeVisits = 0;
};

void RpcRaycastProbeAdd(double micros, double leafTests, double nodeVisits = 0);
void Qu3eRaycastProbeAdd(double micros, double leafTests);
RaycastProbeStats RpcRaycastProbe();
RaycastProbeStats Qu3eRaycastProbe();
void RaycastProbeResetAll();

}  // namespace apex
