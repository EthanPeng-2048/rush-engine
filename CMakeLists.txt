cmake_minimum_required(VERSION 3.28)
project(rush-engine VERSION 0.1.0 LANGUAGES CXX)

# C++23: exstl needs std::expected; named modules land in a later phase.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

# --- Third-party: exstl (header-only, vendored) ------------------------------
add_subdirectory(third_party/exstl)

# --- Rush Engine --------------------------------------------------------------
# Public surface today: core Result alias + version. Physics/render/events
# modules attach here as the engine grows (import rush.engine.*).
add_library(rush-engine STATIC
    src/core/version.cpp
)
add_library(rush::engine ALIAS rush-engine)

target_include_directories(rush-engine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rush-engine PUBLIC exstl::exstl)
target_compile_features(rush-engine PUBLIC cxx_std_23)

# --- Tests (only when built standalone) ---------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    enable_testing()
endif()
