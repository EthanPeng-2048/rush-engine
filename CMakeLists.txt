cmake_minimum_required(VERSION 3.28)
project(rush-engine VERSION 0.1.0 LANGUAGES CXX C)

# C++23: exstl needs std::expected; public API is C++20 named modules.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

# --- Third-party (L1 flat siblings; no nested exSTL) --------------------------
# Order here only creates CMake targets; kvrec must see exstl::exstl already
# defined so it does not pull a second copy.
add_subdirectory(third_party/exstl)
add_subdirectory(third_party/kvrec)
add_subdirectory(third_party/qu3e)
add_subdirectory(third_party/rush-physics-core)

# nudge: C23 unity build of the engine core.
add_library(nudge STATIC src/physics/nudge_unity.c)
target_include_directories(nudge PUBLIC third_party/nudge)
set_target_properties(nudge PROPERTIES C_STANDARD 23 C_STANDARD_REQUIRED ON)
if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    if(EMSCRIPTEN)
        target_compile_options(nudge PRIVATE -std=c23 -ffp-contract=off -msse4.1 -msimd128)
    else()
        target_compile_options(nudge PRIVATE -std=c23 -ffp-contract=off -msse4.1)
    endif()
elseif(MSVC)
    target_compile_options(nudge PRIVATE /std:clatest /experimental:c11atomics /fp:precise)
endif()

# raylib: host (Apex Rush) provides the target when embedding the engine.
set(RUSH_ENGINE_RAYLIB_TARGET "" CACHE STRING "Externally provided raylib target")

# --- Rush Engine --------------------------------------------------------------
# Public C++20 modules:
#   import rush.engine.core;      math, config, paths, Result
#   import rush.engine.physics;   IPhysicsWorld + qu3e/nudge/rush-physics-core
#   import rush.engine.render;    IRenderer + raylib/null backends
add_library(rush-engine STATIC
    src/core/config_impl.cpp
    src/core/map_io.cpp
    src/core/map_csv.cpp
    src/core/paths_impl.cpp
    src/core/version.cpp
    src/job/job_impl.cpp
    src/physics/qu3e_impl.cpp
    src/physics/nudge_impl.cpp
    src/physics/rpc_impl.cpp
    src/physics/vehicle_impl.cpp
    src/physics/collision_bus.cpp
    src/render/null_impl.cpp
)
target_sources(rush-engine PUBLIC
    FILE_SET CXX_MODULES
    FILES
        modules/core.cppm
        modules/physics.cppm
        modules/render.cppm
        modules/events.cppm
        modules/job.cppm
        modules/runtime.cppm
)

add_library(rush::engine ALIAS rush-engine)

target_include_directories(rush-engine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rush-engine
    PUBLIC exstl::exstl qu3e nudge rush-physics-core
    # kvrec is implementation-only (map_io/config_impl); keep it off the
    # public interface so game TUs cannot include <kvrec/...> by accident.
    PRIVATE kvrec::kvrec
)
target_compile_features(rush-engine PUBLIC cxx_std_23)

# raylib backend is optional: only compile when the host provides raylib.
if(RUSH_ENGINE_RAYLIB_TARGET)
    target_sources(rush-engine PRIVATE src/render/raylib_impl.cpp)
    target_link_libraries(rush-engine PRIVATE ${RUSH_ENGINE_RAYLIB_TARGET})
endif()

if(WIN32)
    target_link_options(rush-engine PUBLIC "LINKER:/STACK:8388608")
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    enable_testing()
endif()
