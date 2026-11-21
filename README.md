# Rush Engine

Native async multithreaded game engine: rendering, physics, and a fully
asynchronous event-driven runtime. Used by [Apex Rush Online](https://github.com/EthanPeng-2048/Apex-Rush-Online).

## Layout

```
include/rush/     public headers (module surface grows: core, events, job, runtime, physics, render)
src/              implementation
third_party/      vendored dependencies (exstl)
```

## Build

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

Requires C++23 (`std::expected`). Header-only [exstl](https://github.com/neuralnet-cpp-dev/exstl) is vendored under `third_party/exstl`.

## License

See repository ownership; exstl retains its own license under `third_party/exstl/LICENSE`.
