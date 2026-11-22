# Rush Engine

Native async multithreaded game engine: rendering, physics, and a fully
asynchronous event-driven runtime. Used by [Apex Rush Online](https://github.com/EthanPeng-2048/Apex-Rush-Online).

## Layout

```
modules/          C++20 named modules: rush.engine.{core,physics,render,events,job,runtime}
src/              implementation (physics adapters, vehicle, config/map IO, jobs)
include/rush/     umbrella headers where needed (e.g. rush/rush.h)
third_party/      L1 flat siblings: exstl · kvrec · qu3e · nudge · rush-physics-core
```

Layering (host repo docs: `../docs/architecture.md`):

- **L2** owns vehicle dynamics, `IPhysicsWorld` adapters, `rush::Result`, collision event bus, sim thread.
- **L1** is collision-only (and storage libs); no vehicle code sinks into physics forks.

## Build

Typical host builds use the parent presets (`release` / `debug` / `debug-tools` / `web`).
Standalone:

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

Requires C++23 (`std::expected`) and module scanning. [exstl](https://github.com/neuralnet-cpp-dev/exstl) is vendored under `third_party/exstl`.

## Physics backends

Configured by the game via `[physics] engine`:

| Name | Adapter | Notes |
|------|---------|--------|
| qu3e (default) | `Qu3eWorld` | Stock qu3e |
| nudge | `NudgeWorld` | Feature superset; much slower in bench |
| rpc | `RushPhysicsCore` | `rush-physics-core` fork (`rpc*` symbols) |

## License

See repository ownership; third-party licenses under each `third_party/*` tree.
