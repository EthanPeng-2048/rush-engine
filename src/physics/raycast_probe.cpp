// TEMPORARY diagnostic probe — removed before the perf commit lands.
#include "raycast_probe.h"

namespace apex {

namespace {
RaycastProbeStats g_rpc;
RaycastProbeStats g_qu3e;
}  // namespace

void RpcRaycastProbeAdd(double micros, double leafTests, double nodeVisits) {
    g_rpc.calls += 1;
    g_rpc.micros += micros;
    g_rpc.leafTests += static_cast<uint64_t>(leafTests);
    g_rpc.nodeVisits += static_cast<uint64_t>(nodeVisits);
}

void Qu3eRaycastProbeAdd(double micros, double leafTests) {
    g_qu3e.calls += 1;
    g_qu3e.micros += micros;
    g_qu3e.leafTests += static_cast<uint64_t>(leafTests);
}

RaycastProbeStats RpcRaycastProbe() { return g_rpc; }
RaycastProbeStats Qu3eRaycastProbe() { return g_qu3e; }

void RaycastProbeResetAll() {
    g_rpc = RaycastProbeStats{};
    g_qu3e = RaycastProbeStats{};
}

}  // namespace apex
