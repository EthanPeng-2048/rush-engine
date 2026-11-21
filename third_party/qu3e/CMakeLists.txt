cmake_minimum_required(VERSION 3.20)
project(qu3e CXX)

add_library(qu3e STATIC
    broadphase/q3BroadPhase.cpp
    broadphase/q3DynamicAABBTree.cpp
    collision/q3Box.cpp
    collision/q3Collide.cpp
    common/q3Geometry.cpp
    common/q3Memory.cpp
    dynamics/q3Body.cpp
    dynamics/q3Contact.cpp
    dynamics/q3ContactManager.cpp
    dynamics/q3ContactSolver.cpp
    dynamics/q3Island.cpp
    math/q3Mat3.cpp
    math/q3Quaternion.cpp
    math/q3Vec3.cpp
    scene/q3Scene.cpp
)

target_include_directories(qu3e PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(qu3e PUBLIC cxx_std_11)
