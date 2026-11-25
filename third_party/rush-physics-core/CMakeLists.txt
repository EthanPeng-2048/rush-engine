cmake_minimum_required(VERSION 3.20)
project(rush-physics-core CXX)

# Fork of qu3e (Randy Gaul) with symbols renamed q3* -> rpc* so it can link
# alongside the stock qu3e library. Altered source: identifier prefix only.
add_library(rush-physics-core STATIC
    broadphase/rpcBroadPhase.cpp
    broadphase/rpcDynamicAABBTree.cpp
    broadphase/rpcDynamicSweep.cpp
    broadphase/rpcDynamicTreeRayIndex.cpp
    broadphase/rpcRayIndex.cpp
    broadphase/rpcStaticGrid.cpp
    collision/rpcBox.cpp
    collision/rpcCollide.cpp
    common/rpcGeometry.cpp
    common/rpcMemory.cpp
    common/rpcWorkerPool.cpp
    dynamics/rpcBody.cpp
    dynamics/rpcContact.cpp
    dynamics/rpcContactManager.cpp
    dynamics/rpcContactSolver.cpp
    dynamics/rpcIsland.cpp
    math/rpcMat3.cpp
    math/rpcQuaternion.cpp
    math/rpcVec3.cpp
    scene/rpcScene.cpp
)

target_include_directories(rush-physics-core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(rush-physics-core PUBLIC cxx_std_11)
