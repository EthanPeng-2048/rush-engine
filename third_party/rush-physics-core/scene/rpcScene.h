//--------------------------------------------------------------------------------------------------
/**
@file	rpcScene.h

@author	Randy Gaul
@date	10/10/2014

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

#ifndef RPCSCENE_H
#define RPCSCENE_H

#include <stdio.h>
#include <vector>

#include "../common/rpcSettings.h"
#include "../common/rpcMemory.h"
#include "../dynamics/rpcContactManager.h"
#include "../broadphase/rpcRayIndex.h"

//--------------------------------------------------------------------------------------------------
// Which ray-query acceleration the scene uses.
//   Tree    : the broadphase BVH (reference implementation, one tree for all)
//   Indexed : static geometry in a uniform grid + dynamic bodies in a linear
//             sweep (the shipped default; ~4x cheaper for suspension rays)
//--------------------------------------------------------------------------------------------------
enum class rpcRayStrategy
{
	Tree,
	Indexed
};

// Dynamic-side accelerator used by the Indexed strategy.
//   Sweep : linear scan over body proxy AABBs (wins with few dynamic bodies)
//   Tree  : dedicated dynamic-only BVH (scales with the population)
enum class rpcDynamicRay
{
	Sweep,
	Tree
};

//--------------------------------------------------------------------------------------------------
// rpcSceneConfig
//--------------------------------------------------------------------------------------------------
struct rpcSceneConfig
{
	rpcSceneConfig( r32 stepDt )
		: dt( stepDt )
		, gravity( r32( 0.0 ), r32( -9.81 ), r32( 0.0 ) )
		, iterations( 20 )
		, rayStrategy( rpcRayStrategy::Indexed )
		, dynamicRay( rpcDynamicRay::Tree )
		, staticGridCell( r32( 2.0 ) )
		, rayFactory( nullptr )
	{
	}

	r32 dt;
	rpcVec3 gravity;
	i32 iterations;
	rpcRayStrategy rayStrategy;
	rpcDynamicRay dynamicRay;   // dynamic-side accelerator (Indexed strategy)
	r32 staticGridCell;   // metres per grid cell along X/Z
	// Optional injection (tests / alternate accelerators). When set, it wins
	// over `rayStrategy`; the scene takes ownership of what it creates.
	rpcRayIndexFactory* rayFactory;
};

//--------------------------------------------------------------------------------------------------
// rpcScene
//--------------------------------------------------------------------------------------------------
class rpcBody;
class rpcStaticGrid;
class rpcDynamicSweep;
struct rpcBodyDef;
struct rpcContactConstraint;
struct rpcContactConstraintState;
struct rpcVelocityState;
class rpcRender;
struct rpcIsland;
class rpcWorkerPool;
// Island-solve worker task (D9): Initialize/Solve one prebuilt island. The
// islands share no writable state, so parallel execution is bit-identical
// to serial — only the order changes.
void rpcSolveIslandTask( i32 index, void* arg );

// This listener is used to gather information about two shapes colliding. This
// can be used for game logic and sounds. Physics objects created in these
// callbacks will not be reported until the following frame. These callbacks
// can be called frequently, so make them efficient.
class rpcContactListener
{
public:
	virtual ~rpcContactListener( )
	{
	}

	virtual void BeginContact( const rpcContactConstraint *contact ) = 0;
	virtual void EndContact( const rpcContactConstraint *contact ) = 0;
};

// This class represents general queries for points, AABBs and Raycasting.
// ReportShape is called the moment a valid shape is found. The return
// value of ReportShape controls whether to continue or stop the query.
// By returning only true, all shapes that fulfill the query will be re-
// ported.
class rpcQueryCallback
{
public:
	virtual ~rpcQueryCallback( )
	{
	}

	virtual bool ReportShape( rpcBox *box ) = 0;

	// Batch-raycast callback: box, the ray index within the batch, and the
	// ray's per-ray data (toi/normal freshly written by the box test). The
	// default forwards to ReportShape( box ) so single-ray subclasses keep
	// working unchanged.
	virtual bool ReportShape( rpcBox *box, i32 rayIndex, rpcRaycastData* ray )
	{
		RPC_UNUSED( rayIndex );
		RPC_UNUSED( ray );
		return ReportShape( box );
	}
};

class rpcScene : public rpcDynamicBodySource
{
public:
	rpcScene( r32 dt, const rpcVec3& gravity = rpcVec3( r32( 0.0 ), r32( -9.8 ), r32( 0.0 ) ), i32 iterations = 20 );
	explicit rpcScene( const rpcSceneConfig& config );
	~rpcScene( );

	// Run the simulation forward in time by dt (fixed timestep). Variable
	// timestep is not supported.
	void Step( );

	// Construct a new rigid body. The BodyDef can be reused at the user's
	// discretion, as no reference to the BodyDef is kept.
	rpcBody* CreateBody( const rpcBodyDef& def );

	// Frees a body, removes all shapes associated with the body and frees
	// all shapes and contacts associated and attached to this body.
	void RemoveBody( rpcBody* body );
	void RemoveAllBodies( );

	// Enables or disables rigid body sleeping. Sleeping is an effective CPU
	// optimization where bodies are put to sleep if they don't move much.
	// Sleeping bodies sit in memory without being updated, until the are
	// touched by something that wakes them up. The default is enabled.
	void SetAllowSleep( bool allowSleep );

	// Increasing the iteration count increases the CPU cost of simulating
	// Scene.Step(). Decreasing the iterations makes the simulation less
	// realistic (convergent). A good iteration number range is 5 to 20.
	// Only positive numbers are accepted. Non-positive and negative
	// inputs set the iteration count to 1.
	void SetIterations( i32 iterations );

	// Friction occurs when two rigid bodies have shapes that slide along one
	// another. The friction force resists this sliding motion.
	void SetEnableFriction( bool enabled );

	// Parallel island solving (D9): solves every island's velocity/position
	// constraints across a worker pool. Islands share no writable state, so
	// results are bit-identical to serial solving; only the execution order
	// changes. Default OFF — enable only outside deterministic paths
	// (drive-test stays serial).
	void SetParallelIslands( bool enabled );
	bool ParallelIslands( ) const { return m_parallelIslands; }

	// Render the scene with an interpolated time between the last frame and
	// the current simulation step.
	void Render( rpcRender* render ) const;

	// Per-step instrumentation for profiling and stress tests. Counts are
	// cheap enough to maintain always; the timers only run when enabled, so
	// gameplay pays nothing for this when it is off.
	struct Stats
	{
		i32 contacts;
		i32 contactPoints;
		i32 islands;
		i32 awakeBodies;
		double narrowphaseMs;
		double broadphaseMs;
		double islandBuildMs;
		double islandSolveMs;
		double syncMs;
		size_t poolContactStates;  // island-pool capacity, in states
		size_t poolBytes;          // total reserved island-pool memory
	};
	void SetStatsEnabled( bool on ) { m_statsEnabled = on; }
	// Deep counters walk every contact/body per step: tests and offline probes
	// only, never the benchmark path.
	void SetDeepStats( bool on ) { m_deepStats = on; }
	const Stats& GetStats( ) const { return m_stats; }

	// Broadphase tree shape/traversal counters (see rpcDynamicAABBTree).
	const rpcDynamicAABBTree& BroadphaseTree( ) const { return m_contactManager.m_broadphase.m_tree; }

	// rpcDynamicBodySource — population for the dynamic ray sweep.
	rpcBody** Bodies( ) override { return m_dynamicBodies.data( ); }
	i32 BodyCount( ) const override { return (i32)m_dynamicBodies.size( ); }

	rpcRayStrategy RayStrategy( ) const { return m_rayStrategy; }
	// Static ray-grid cell count (0 when the grid is not the active strategy).
	i32 StaticGridCells( ) const { return m_staticRayIndex ? m_staticRayIndex->CellCount( ) : 0; }
	// Monotonic per-scene shape id. Closest-hit selection breaks exact distance
	// ties by it, so every ray strategy returns the same shape for the same ray.
	i32 NextBoxUid( ) { return ++m_boxUidCounter; }
	// Shape-proxy hooks (called by rpcBody): route insert/remove/move of a shape
	// to the ray accelerator that owns it.
	void RegisterBoxProxy( rpcBox* box, const rpcAABB& fatAabb );
	void UnregisterBoxProxy( rpcBox* box );
	void UpdateBoxProxy( rpcBox* box, const rpcAABB& fatAabb );

	// Gets and sets the global gravity vector used during integration
	const rpcVec3 GetGravity( ) const;
	void SetGravity( const rpcVec3& gravity );

	// Removes all bodies from the scene.
	void Shutdown( );

	// Sets the listener to report collision start/end. Provides the user
	// with a pointer to an rpcContactConstraint. The rpcContactConstraint
	// holds pointers to the two shapes involved in a collision, and the
	// two bodies connected to each shape. The rpcContactListener will be
	// called very often, so it is recommended for the funciton to be very
	// efficient. Provide a NULL pointer to remove the previously set
	// listener.
	void SetContactListener( rpcContactListener* listener );

	// Query the world to find any shapes that can potentially intersect
	// the provided AABB. This works by querying the broadphase with an
	// AAABB -- only *potential* intersections are reported. Perhaps the
	// user might use lmDistance as fine-grained collision detection.
	void QueryAABB( rpcQueryCallback *cb, const rpcAABB& aabb ) const;

	// Query the world to find any shapes intersecting a world space point.
	void QueryPoint( rpcQueryCallback *cb, const rpcVec3& point ) const;

	// Query the world to find any shapes intersecting a ray.
	void RayCast( rpcQueryCallback *cb, rpcRaycastData& rayCast, const rpcBody* ignore = nullptr ) const;

	// Batch form of RayCast: `count` rays (<= 32) sharing one tree traversal.
	// The callback sees ReportShape( box, rayIndex, ray ) per hit, so closest-
	// hit selection can be tracked per ray. Results are identical to count
	// separate RayCast calls; only the traversal is shared.
	void RayCastBatch( rpcQueryCallback *cb, rpcRaycastData* rayCasts, i32 count,
	                   const rpcBody* ignore = nullptr ) const;

	// Dump all rigid bodies and shapes into a log file. The log can be
	// used as C++ code to re-create an initial scene setup. Contacts
	// are *not* logged, meaning any cached resolution solutions will
	// not be saved to the log file. This means the log file will be most
	// accurate when dumped upon scene initialization, instead of mid-
	// simulation.
	void Dump( FILE* file ) const;

private:
	// Step pipeline stages (rpc-refactor.md D6): one concern per method.
	void SolveIslands( );	// seed loop: BuildIsland per seed, then Initialize/Solve
	void SolveIslandsBatched( );	// build-all islands, then solve (serial or pooled)
	void SyncAndFind( );	// SynchronizeProxies + FindNewContacts + clear forces

	// DFS the constraint graph from seed into the working island. Stack must
	// hold at least m_bodyCount entries.
	void BuildIsland( rpcBody* seed, rpcIsland& island, rpcBody** stack, i32 stackSize );

	// Contact scan for the stats counter (never on the hot path when off).
	void CollectStats( i32 islands );

	// Island batch pools. Every island's body/contact lists are appended in
	// island order at exact offsets, so the pools are sized to what the step
	// actually built — never to islands × scene caps.
	struct IslandSlice
	{
		size_t bodyFirst;
		size_t bodyCount;
		size_t contactFirst;
		size_t contactCount;
	};
	void EnsureScratch( size_t bodies, size_t contacts );
	void EnsurePools( size_t bodies, size_t contacts );
	// (Re)attach an island's spans after a pool (re)allocation.
	void PointIsland( i32 index );
	void BuildAndAppendIsland( rpcBody* seed, i32 index );

	rpcContactManager m_contactManager;
	rpcPagedAllocator m_boxAllocator;

	i32 m_bodyCount;
	rpcBody* m_bodyList;
	i32 m_dynamicCount;   // cached (CreateBody/RemoveBody maintain it)
	i32 m_contactPeak;    // pool sizing upper bound (monotone, never shrinks)

	rpcStack m_stack;
	rpcHeap m_heap;

	rpcVec3 m_gravity;
	r32 m_dt;
	i32 m_iterations;

	bool m_newBox;
	bool m_allowSleep;
	bool m_enableFriction;

	// Parallel island solving (D9): lazily-built worker pool; the pool is
	// only touched when parallel solving is enabled (never in deterministic
	// paths).
	bool m_parallelIslands;
	rpcWorkerPool* m_workers;

	// Island batch pools (indexed by island, appended in build order). Each
	// island's slices are exact: bodies/contacts are copied in from the build
	// scratch, velocities/states are 1:1 work arrays.
	std::vector<rpcBody*> m_poolBodies;
	std::vector<rpcVelocityState> m_poolVelocities;
	std::vector<rpcContactConstraint*> m_poolContacts;
	std::vector<rpcContactConstraintState> m_poolContactStates;
	std::vector<rpcIsland> m_islands;   // island slots (indexed by seed)
	std::vector<IslandSlice> m_slices;  // per-island pool offsets
	std::vector<rpcBody*> m_dfsStack;   // island-build DFS stack (reused)
	// Build scratch: one island's lists at a time, reused across islands.
	std::vector<rpcBody*> m_scratchBodies;
	std::vector<rpcContactConstraint*> m_scratchContacts;

	i32 m_boxUidCounter;   // monotonic shape id (closest-hit tie-break)
	bool m_deepStats;      // opt-in expensive counters (contact points, awake)
	bool m_statsEnabled;
	Stats m_stats;

	// Ray-query acceleration (see rpcRayStrategy). `Tree` keeps the historical
	// single-BVH path; `Indexed` splits static geometry (grid) from dynamic
	// bodies (linear sweep). Both are behind rpcRayIndex, so the query path
	// never branches on the strategy at run time.
	rpcRayStrategy m_rayStrategy;
	rpcRayIndex* m_staticRayIndex;
	rpcRayIndex* m_dynamicRayIndex;
	std::vector<rpcBody*> m_dynamicBodies;   // sweep population (dense)

	void InitRayIndexes( const rpcSceneConfig& config );
	void ShutdownRayIndexes( );

	friend class rpcBody;
	friend void rpcSolveIslandTask( i32 index, void* arg );
};

#endif // RPCSCENE_H
