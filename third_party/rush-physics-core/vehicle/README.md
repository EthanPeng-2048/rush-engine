# Rush Physics Core — vehicle (retired plan)

**Decision: vehicle dynamics do NOT live in rush-physics-core.**

This directory is a retired placeholder. There is **no plan** to extract
TireModel / suspension / powertrain into L1, and **no plan** for
vehicle+collision co-simulation inside the RPC engine (no multi-island
vehicle step, no parallel vehicle updates in the physics library).

## Boundary (durable)

| Layer | Owns |
|---|---|
| L1 `rush-physics-core` (`rpc*`) | Collision only: broadphase, box narrowphase, contact islands, sequential-impulse solver, raycast primitives |
| L2 `rush-engine` | `IPhysicsWorld` adapters + **all** vehicle dynamics (`vehicle_impl.cpp`, `TireModel`, suspension raycasts, powertrain) |

Vehicles drive any backend (qu3e / nudge / rpc) only through
`IPhysicsWorld` (bodies, raycast, forces, friction). RPC remains the
deep-customization target for **collision** (island step, solver, queries),
not for tire models.

Peer backends Qu3eWorld / NudgeWorld share the same Vehicle API; swapping
`[physics] engine` never moves vehicle code into a physics library.
