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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <chrono>

#include "rpcScene.h"
#include "../dynamics/rpcBody.h"
#include "../dynamics/rpcIsland.h"
#include "../dynamics/rpcContact.h"
#include "../dynamics/rpcContactSolver.h"
#include "../collision/rpcBox.h"
#include "../common/rpcWorkerPool.h"
#include "../broadphase/rpcStaticGrid.h"
#include "../broadphase/rpcDynamicSweep.h"
#include "../broadphase/rpcDynamicTreeRayIndex.h"

namespace {

using StatsClock = std::chrono::steady_clock;

double ElapsedMs( StatsClock::time_point from, StatsClock::time_point to )
{
	return std::chrono::duration<double, std::milli>( to - from ).count( );
}

}  // namespace

//--------------------------------------------------------------------------------------------------
// rpcScene
//--------------------------------------------------------------------------------------------------
rpcScene::rpcScene( r32 dt, const rpcVec3& gravity, i32 iterations )
	: m_contactManager( &m_stack )
	, m_boxAllocator( sizeof( rpcBox ), 256 )
	, m_bodyCount( 0 )
	, m_dynamicCount( 0 )
	, m_contactPeak( 0 )
	, m_bodyList( NULL )
	, m_gravity( gravity )
	, m_dt( dt )
	, m_iterations( iterations )
	, m_newBox( false )
	, m_allowSleep( true )
	, m_enableFriction( true )
	, m_parallelIslands( false )
	, m_workers( NULL )
	, m_boxUidCounter( 0 )
	, m_deepStats( false )
	, m_statsEnabled( false )
	, m_stats( )
	, m_rayStrategy( rpcRayStrategy::Tree )
	, m_staticRayIndex( NULL )
	, m_dynamicRayIndex( NULL )
{
	InitRayIndexes( rpcSceneConfig( dt ) );
}

//--------------------------------------------------------------------------------------------------
rpcScene::rpcScene( const rpcSceneConfig& config )
	: m_contactManager( &m_stack )
	, m_boxAllocator( sizeof( rpcBox ), 256 )
	, m_bodyCount( 0 )
	, m_dynamicCount( 0 )
	, m_contactPeak( 0 )
	, m_bodyList( NULL )
	, m_gravity( config.gravity )
	, m_dt( config.dt )
	, m_iterations( config.iterations )
	, m_newBox( false )
	, m_allowSleep( true )
	, m_enableFriction( true )
	, m_parallelIslands( false )
	, m_workers( NULL )
	, m_boxUidCounter( 0 )
	, m_deepStats( false )
	, m_statsEnabled( false )
	, m_stats( )
	, m_rayStrategy( config.rayStrategy )
	, m_staticRayIndex( NULL )
	, m_dynamicRayIndex( NULL )
{
	InitRayIndexes( config );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::InitRayIndexes( const rpcSceneConfig& config )
{
	ShutdownRayIndexes( );

	// Injected accelerators take precedence: the scene only ever talks to the
	// rpcRayIndex interface, so a different implementation changes nothing else.
	if ( config.rayFactory )
	{
		m_rayStrategy = rpcRayStrategy::Indexed;
		m_staticRayIndex = config.rayFactory->CreateStaticIndex( config.staticGridCell );
		m_dynamicRayIndex = config.rayFactory->CreateDynamicIndex( this );
		return;
	}

	if ( config.rayStrategy != rpcRayStrategy::Indexed )
		return;

	m_rayStrategy = rpcRayStrategy::Indexed;
	m_staticRayIndex = new rpcStaticGrid( config.staticGridCell );
	// Dynamic side: a dedicated BVH scales with the population; the linear sweep
	// is kept as the alternative for small fields (see rpcDynamicRay). Measured
	// on 120 cars: sweep cost grows ~linearly with the car count (0.95 us/ray at
	// 24 cars -> 2.0 us/ray at 120), the dynamic tree stays near flat, which is
	// why Tree is the default.
	m_dynamicRayIndex = config.dynamicRay == rpcDynamicRay::Tree
	                        ? static_cast<rpcRayIndex*>( new rpcDynamicTreeRayIndex( ) )
	                        : static_cast<rpcRayIndex*>( new rpcDynamicSweep( this ) );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::ShutdownRayIndexes( )
{
	delete m_staticRayIndex;
	delete m_dynamicRayIndex;
	m_staticRayIndex = NULL;
	m_dynamicRayIndex = NULL;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::RegisterBoxProxy( rpcBox* box, const rpcAABB& fatAabb )
{
	if ( m_rayStrategy == rpcRayStrategy::Indexed && m_staticRayIndex && box->body &&
	     ( box->body->m_flags & rpcBody::eStatic ) )
		m_staticRayIndex->Add( box, fatAabb );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::UnregisterBoxProxy( rpcBox* box )
{
	if ( m_staticRayIndex )
		m_staticRayIndex->Remove( box );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::UpdateBoxProxy( rpcBox* box, const rpcAABB& fatAabb )
{
	// Only static geometry lives in the grid; dynamic bodies are enumerated by
	// the sweep through rpcBody::GetFatAABB, so a moving dynamic shape costs
	// nothing here. Static shapes are immutable after creation (the track is
	// built once), which is what makes the grid a pure build-once structure.
	if ( m_staticRayIndex && box->body && ( box->body->m_flags & rpcBody::eStatic ) )
		m_staticRayIndex->Update( box, fatAabb );
}

//--------------------------------------------------------------------------------------------------
rpcScene::~rpcScene( )
{
	ShutdownRayIndexes( );
	Shutdown( );
	// m_workers is the process-wide shared singleton (rpcWorkerPool::Shared)
	// — it outlives every scene and must never be deleted here.
}

//--------------------------------------------------------------------------------------------------
void rpcScene::Step( )
{
	StatsClock::time_point mark;
	if ( m_statsEnabled )
		mark = StatsClock::now( );

	if ( m_newBox )
	{
		m_contactManager.m_broadphase.UpdatePairs( );
		m_newBox = false;
	}

	if ( m_statsEnabled )
	{
		const StatsClock::time_point now = StatsClock::now( );
		m_stats.broadphaseMs = ElapsedMs( mark, now );
		mark = now;
	}

	m_contactManager.TestCollisions( );

	if ( m_statsEnabled )
	{
		const StatsClock::time_point now = StatsClock::now( );
		m_stats.narrowphaseMs = ElapsedMs( mark, now );
		mark = now;
	}

	SolveIslands( );
	SyncAndFind( );

	if ( m_statsEnabled )
	{
		const StatsClock::time_point now = StatsClock::now( );
		m_stats.syncMs = ElapsedMs( mark, now ) - m_stats.islandBuildMs - m_stats.islandSolveMs;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SolveIslands( )
{
	// Batch path: build every island up front, then solve (serial or across
	// the worker pool — bit-identical numerics, islands share no writable
	// state). Static track bodies never size the pools, so steady-state
	// solving allocates and clears nothing.
	SolveIslandsBatched( );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SolveIslandsBatched( )
{
	StatsClock::time_point mark;
	if ( m_statsEnabled )
		mark = StatsClock::now( );

		// island build (shared statics must re-join later islands), so they
	// never leak across steps — clearing them here would walk 6000+ track
	// bodies per step for nothing. Iterate the dense dynamic list: walking
	// m_bodyList would touch every static body's header (296 B apart, scattered
	// by the heap) just to skip it — measured as ~0.1 ms/substep on a track.
	for ( rpcBody* body : m_dynamicBodies )
		body->m_flags &= ~rpcBody::eIsland;

	// Island body lists are built one island at a time into the scratch
	// buffers, then copied into the step pools at exact offsets. Only the
	// largest single island has to fit the scratch, so both buffers size
	// against the scene itself.
	m_contactPeak = rpcMax( m_contactPeak, m_contactManager.m_contactCount );
	EnsureScratch( (size_t)m_bodyCount, (size_t)rpcMax( 1, m_contactPeak ) );
	EnsurePools( 0, 0 );

	const size_t maxIslands = (size_t)rpcMax( 1, m_dynamicCount );
	if ( m_dfsStack.size( ) < (size_t)m_bodyCount )
		m_dfsStack.resize( (size_t)m_bodyCount );
	if ( m_islands.size( ) < maxIslands )
		m_islands.resize( maxIslands );
	if ( m_slices.size( ) < maxIslands )
		m_slices.resize( maxIslands );

	// Build every island and Initialize() right away: Initialize snapshots
	// body->m_islandIndex, which a shared static body would overwrite if the
	// islands were built first and initialized later. Seeds come from the dense
	// dynamic list — statics can never seed an island, so iterating them only
	// costs cache misses walking the linked body list.
	i32 islandCount = 0;
	for ( rpcBody* seed : m_dynamicBodies )
	{
		// Seed cannot be apart of an island already
		if ( seed->m_flags & rpcBody::eIsland )
			continue;

		// Seed must be awake
		if ( !(seed->m_flags & rpcBody::eAwake) )
			continue;

		BuildAndAppendIsland( seed, islandCount );
		++islandCount;
	}

	if ( m_statsEnabled )
	{
		const StatsClock::time_point now = StatsClock::now( );
		m_stats.islandBuildMs = ElapsedMs( mark, now );
		mark = now;
	}

	// Solve all islands. Parallel and serial execution are bit-identical:
	// islands share no writable state (static bodies are only read in
	// rpcIsland::Solve). Gate on workload: tiny batches lose more to
	// synchronization than they gain (16-car grids stay serial). The pool is
	// the process-wide singleton — never created on this hot path.
	if ( !m_parallelIslands || !m_workers )
	{
		for ( i32 i = 0; i < islandCount; ++i )
			m_islands[ (size_t)i ].Solve( );
	}
	else
	{
		const i32 workers = (i32)m_workers->WorkerCount( ) + 1;
		if ( islandCount < 4 * workers )
		{
			for ( i32 i = 0; i < islandCount; ++i )
				m_islands[ (size_t)i ].Solve( );
		}
		else
		{
			m_workers->Run( rpcSolveIslandTask, this, islandCount );
		}
	}

	if ( m_statsEnabled )
	{
		m_stats.islandSolveMs = ElapsedMs( mark, StatsClock::now( ) );
		CollectStats( islandCount );
	}
}

//--------------------------------------------------------------------------------------------------
void rpcScene::CollectStats( i32 islands )
{
	i32 points = 0;
	for ( const rpcContactConstraint* c = m_contactManager.m_contactList; c; c = c->next )
	{
		if ( c->m_flags & rpcContactConstraint::eColliding )
			points += c->manifold.contactCount;
	}

	i32 awake = 0;
	for ( rpcBody* body = m_bodyList; body; body = body->m_next )
	{
		if ( body->IsAwake( ) && !( body->m_flags & rpcBody::eStatic ) )
			++awake;
	}

	m_stats.contacts = m_contactManager.m_contactCount;
	m_stats.contactPoints = points;
	m_stats.islands = islands;
	m_stats.awakeBodies = awake;
	m_stats.poolContactStates = m_poolContactStates.capacity( );
	m_stats.poolBytes = m_poolContactStates.capacity( ) * sizeof( rpcContactConstraintState ) +
	                    m_poolVelocities.capacity( ) * sizeof( rpcVelocityState ) +
	                    m_poolBodies.capacity( ) * sizeof( rpcBody* ) +
	                    m_poolContacts.capacity( ) * sizeof( rpcContactConstraint* );
}

//--------------------------------------------------------------------------------------------------
// Pools grow ×2 and never shrink, so steady state allocates nothing. Growth
// invalidates every island's spans, hence the re-point pass.
void rpcScene::EnsureScratch( size_t bodies, size_t contacts )
{
	if ( m_scratchBodies.size( ) < bodies )
		m_scratchBodies.resize( bodies );
	if ( m_scratchContacts.size( ) < contacts )
		m_scratchContacts.resize( contacts );
}

//--------------------------------------------------------------------------------------------------
// Pools grow ×2 and never shrink, so steady state allocates nothing. Growth
// invalidates every island's spans, hence the re-point pass. resize() (not
// reserve()) keeps size() == usable length, so the span writes stay legal.
void rpcScene::EnsurePools( size_t bodies, size_t contacts )
{
	bool moved = false;
	const auto grow = [&moved]( auto& pool, size_t need ) {
		if ( pool.size( ) < need )
		{
			pool.resize( need * 2 );
			moved = true;
		}
	};
	grow( m_poolBodies, bodies );
	grow( m_poolVelocities, bodies );
	grow( m_poolContacts, contacts );
	grow( m_poolContactStates, contacts );

	if ( !moved )
		return;

	// Every island built so far points into the old buffers: re-attach from
	// the recorded offsets (contents survive the move).
	for ( size_t i = 0; i < m_slices.size( ); ++i )
	{
		if ( m_slices[ i ].bodyCount == 0 && m_slices[ i ].contactCount == 0 )
			continue;
		PointIsland( (i32)i );
	}
}

//--------------------------------------------------------------------------------------------------
void rpcScene::PointIsland( i32 index )
{
	const IslandSlice& slice = m_slices[ (size_t)index ];
	rpcIsland& island = m_islands[ (size_t)index ];
	island.Reset( std::span<rpcBody*>( m_poolBodies.data( ) + slice.bodyFirst, slice.bodyCount ),
	              std::span<rpcVelocityState>( m_poolVelocities.data( ) + slice.bodyFirst, slice.bodyCount ),
	              std::span<rpcContactConstraint*>( m_poolContacts.data( ) + slice.contactFirst, slice.contactCount ),
	              std::span<rpcContactConstraintState>( m_poolContactStates.data( ) + slice.contactFirst, slice.contactCount ),
	              m_dt, m_gravity, m_iterations, m_allowSleep, m_enableFriction );
	// Reset re-arms the counts; restore what the build already produced.
	island.m_bodyCount = (i32)slice.bodyCount;
	island.m_contactCount = (i32)slice.contactCount;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::BuildAndAppendIsland( rpcBody* seed, i32 index )
{
	// Build one island into the scratch buffers, then copy it into the step
	// pools at the exact offset. Sizing the pools by islands × scene caps
	// (the previous scheme) multiplied every pool by the dynamic-body count:
	// a car pile-up with tens of thousands of contacts reserved gigabytes.
	rpcIsland scratch;
	scratch.Reset( std::span<rpcBody*>( m_scratchBodies.data( ), m_scratchBodies.size( ) ),
	               std::span<rpcVelocityState>( ),
	               std::span<rpcContactConstraint*>( m_scratchContacts.data( ), m_scratchContacts.size( ) ),
	               std::span<rpcContactConstraintState>( ), m_dt, m_gravity, m_iterations,
	               m_allowSleep, m_enableFriction );
	BuildIsland( seed, scratch, m_dfsStack.data( ), (i32)m_dfsStack.size( ) );

	const size_t bodyFirst = index == 0 ? 0 : m_slices[ (size_t)index - 1 ].bodyFirst +
	                                             m_slices[ (size_t)index - 1 ].bodyCount;
	const size_t contactFirst = index == 0 ? 0 : m_slices[ (size_t)index - 1 ].contactFirst +
	                                                m_slices[ (size_t)index - 1 ].contactCount;
	const size_t bodyCount = (size_t)scratch.m_bodyCount;
	const size_t contactCount = (size_t)scratch.m_contactCount;

	EnsurePools( bodyFirst + bodyCount, contactFirst + contactCount );

	if ( bodyCount )
		std::memcpy( m_poolBodies.data( ) + bodyFirst, scratch.m_bodies.data( ),
		             bodyCount * sizeof( rpcBody* ) );
	if ( contactCount )
		std::memcpy( m_poolContacts.data( ) + contactFirst, scratch.m_contacts.data( ),
		             contactCount * sizeof( rpcContactConstraint* ) );

	m_slices[ (size_t)index ] = { bodyFirst, bodyCount, contactFirst, contactCount };
	PointIsland( index );

	rpcIsland& island = m_islands[ (size_t)index ];

	// Clear the static-body island flags right away: a shared static body
	// (ground, walls) must be re-joinable by later islands, otherwise its
	// m_islandIndex stays stale and Solve() indexes out of bounds. Keep
	// Initialize() before the next island overwrites those indices.
	for ( i32 i = 0; i < island.m_bodyCount; ++i )
	{
		rpcBody* body = island.m_bodies[ i ];
		if ( body->m_flags & rpcBody::eStatic )
			body->m_flags &= ~rpcBody::eIsland;
	}
	island.Initialize( );
}

//--------------------------------------------------------------------------------------------------
// Island-solve worker task (D9): one island per task. Islands are disjoint
// body sets; static bodies appear in several islands but rpcIsland::Solve
// only reads them. Bit-identical to serial solving.
void rpcSolveIslandTask( i32 index, void* arg )
{
	rpcScene* scene = (rpcScene*)arg;
	rpcIsland& island = scene->m_islands[ (size_t)index ];
	island.Solve( );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::BuildIsland( rpcBody* seed, rpcIsland& island, rpcBody** stack, i32 stackSize )
{
	i32 stackCount = 0;
	stack[ stackCount++ ] = seed;

	// Mark seed as apart of island
	seed->m_flags |= rpcBody::eIsland;

	// Perform DFS on constraint graph
	while( stackCount > 0 )
	{
		// Decrement stack to implement iterative backtracking
		rpcBody *body = stack[ --stackCount ];
		island.Add( body );

		// Awaken all bodies connected to the island
		body->SetToAwake( );

		// Do not search across static bodies to keep island
		// formations as small as possible, however the static
		// body itself should be apart of the island in order
		// to properly represent a full contact
		if ( body->m_flags & rpcBody::eStatic )
			continue;

		// Search all contacts connected to this body
		rpcContactEdge* contacts = body->m_contactList;
		for ( rpcContactEdge* edge = contacts; edge; edge = edge->next )
		{
			rpcContactConstraint *contact = edge->constraint;

			// Skip contacts that have been added to an island already
			if ( contact->m_flags & rpcContactConstraint::eIsland )
				continue;

			// Can safely skip this contact if it didn't actually collide with anything
			if ( !(contact->m_flags & rpcContactConstraint::eColliding) )
				continue;

			// Skip sensors
			if ( contact->A->sensor || contact->B->sensor )
				continue;

			// Mark island flag and add to island
			contact->m_flags |= rpcContactConstraint::eIsland;
			island.Add( contact );

			// Attempt to add the other body in the contact to the island
			// to simulate contact awakening propogation
			rpcBody* other = edge->other;
			if ( other->m_flags & rpcBody::eIsland )
				continue;

			assert( stackCount < stackSize );

			stack[ stackCount++ ] = other;
			other->m_flags |= rpcBody::eIsland;
		}
	}
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SyncAndFind( )
{
	StatsClock::time_point mark;
	if ( m_statsEnabled )
		mark = StatsClock::now( );

	// Update the broadphase AABBs (dynamic bodies only; the dense list avoids
	// touching every static body's cache line just to skip it).
	for ( rpcBody* body : m_dynamicBodies )
		body->SynchronizeProxies( );

	if ( m_statsEnabled )
	{
		const StatsClock::time_point now = StatsClock::now( );
		m_stats.proxyMs = ElapsedMs( mark, now );
		mark = now;
	}

	// Look for new contacts
	m_contactManager.FindNewContacts( );

	if ( m_statsEnabled )
		m_stats.pairMs = ElapsedMs( mark, StatsClock::now( ) );

	// Clear all forces — dynamic bodies only: static bodies never receive
	// forces (L2 applies forces to vehicles), and the dense list keeps this
	// walk off the static bodies' cache lines.
	for ( rpcBody* body : m_dynamicBodies )
	{
		rpcIdentity( body->m_force );
		rpcIdentity( body->m_torque );
	}
}

//--------------------------------------------------------------------------------------------------
rpcBody* rpcScene::CreateBody( const rpcBodyDef& def )
{
	rpcBody* body = (rpcBody*)m_heap.Allocate( sizeof( rpcBody ) );
	new (body) rpcBody( def, this );

	// Add body to scene bodyList
	body->m_prev = NULL;
	body->m_next = m_bodyList;

	if ( m_bodyList )
		m_bodyList->m_prev = body;

	m_bodyList = body;
	++m_bodyCount;
	if ( !( body->m_flags & rpcBody::eStatic ) )
	{
		++m_dynamicCount;
		m_dynamicBodies.push_back( body );
	}

	return body;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::RemoveBody( rpcBody* body )
{
	assert( m_bodyCount > 0 );

	m_contactManager.RemoveContactsFromBody( body );

	body->RemoveAllBoxes( );

	// Remove body from scene bodyList
	if ( body->m_next )
		body->m_next->m_prev = body->m_prev;

	if ( body->m_prev )
		body->m_prev->m_next = body->m_next;

	if ( body == m_bodyList )
		m_bodyList = body->m_next;

	--m_bodyCount;
	if ( !( body->m_flags & rpcBody::eStatic ) )
	{
		--m_dynamicCount;
		for ( size_t i = 0; i < m_dynamicBodies.size( ); ++i )
		{
			if ( m_dynamicBodies[ i ] == body )
			{
				m_dynamicBodies[ i ] = m_dynamicBodies.back( );
				m_dynamicBodies.pop_back( );
				break;
			}
		}
	}

	m_heap.Free( body );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::RemoveAllBodies( )
{
	rpcBody* body = m_bodyList;

	while ( body )
	{
		rpcBody* next = body->m_next;

		body->RemoveAllBoxes( );

		m_heap.Free( body );

		body = next;
	}

	m_bodyList = NULL;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SetAllowSleep( bool allowSleep )
{
	m_allowSleep = allowSleep;

	if ( !allowSleep )
	{
		for ( rpcBody* body = m_bodyList; body; body = body->m_next )
			body->SetToAwake( );
	}
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SetIterations( i32 iterations )
{
	m_iterations = rpcMax( 1, iterations );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SetEnableFriction( bool enabled )
{
	m_enableFriction = enabled;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SetParallelIslands( bool enabled )
{
	if ( m_parallelIslands == enabled )
		return;
	m_parallelIslands = enabled;
	// The pool is the process-wide shared singleton (created at startup,
	// never resized/torn down); the flag just switches its use on/off.
	m_workers = enabled ? &rpcWorkerPool::Shared( ) : NULL;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::Render( rpcRender* render ) const
{
	rpcBody* body = m_bodyList;

	while ( body )
	{
		body->Render( render );
		body = body->m_next;
	}

	m_contactManager.RenderContacts( render );
	//m_contactManager.m_broadphase.m_tree.Render( render );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcScene::GetGravity( ) const
{
	return m_gravity;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SetGravity( const rpcVec3& gravity )
{
	m_gravity = gravity;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::Shutdown( )
{
	RemoveAllBodies( );

	m_boxAllocator.Clear( );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::SetContactListener( rpcContactListener* listener )
{
	m_contactManager.m_contactListener = listener;
}

//--------------------------------------------------------------------------------------------------
void rpcScene::QueryAABB( rpcQueryCallback *cb, const rpcAABB& aabb ) const
{
	struct SceneQueryWrapper
	{
		bool TreeCallBack( i32 id )
		{
			rpcAABB aabb;
			rpcBox *box = (rpcBox *)broadPhase->m_tree.GetUserData( id );

			box->ComputeAABB( box->body->GetTransform( ), &aabb );

			if ( rpcAABBtoAABB( m_aabb, aabb ) )
			{
				return cb->ReportShape( box );
			}

			return true;
		}

		rpcQueryCallback *cb;
		const rpcBroadPhase *broadPhase;
		rpcAABB m_aabb;
	};

	SceneQueryWrapper wrapper;
	wrapper.m_aabb = aabb;
	wrapper.broadPhase = &m_contactManager.m_broadphase;
	wrapper.cb = cb;
	m_contactManager.m_broadphase.m_tree.Query( &wrapper, aabb );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::QueryPoint( rpcQueryCallback *cb, const rpcVec3& point ) const
{
	struct SceneQueryWrapper
	{
		bool TreeCallBack( i32 id )
		{
			rpcBox *box = (rpcBox *)broadPhase->m_tree.GetUserData( id );

			if ( box->TestPoint( box->body->GetTransform( ), m_point ) )
			{
				cb->ReportShape( box );
			}

			return true;
		}

		rpcQueryCallback *cb;
		const rpcBroadPhase *broadPhase;
		rpcVec3 m_point;
	};

	SceneQueryWrapper wrapper;
	wrapper.m_point = point;
	wrapper.broadPhase = &m_contactManager.m_broadphase;
	wrapper.cb = cb;
	const r32 k_fattener = r32( 0.5 );
	rpcVec3 v( k_fattener, k_fattener, k_fattener );
	rpcAABB aabb;
	aabb.min = point - v;
	aabb.max = point + v;
	m_contactManager.m_broadphase.m_tree.Query( &wrapper, aabb );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::RayCast( rpcQueryCallback *cb, rpcRaycastData& rayCast, const rpcBody* ignore ) const
{
	// Indexed strategy: static geometry (grid) answers the common case — the
	// suspension ray hits the road under the wheel — and its hit tightens
	// rayCast.t before the dynamic pass, which then prunes against it.
	if ( m_rayStrategy == rpcRayStrategy::Indexed && m_staticRayIndex && m_dynamicRayIndex )
	{
		m_staticRayIndex->Raycast( cb, rayCast );
		m_dynamicRayIndex->Raycast( cb, rayCast );
		return;
	}

	struct SceneQueryWrapper
	{
		bool TreeCallBack( i32 id )
		{
			rpcBox *box = (rpcBox *)broadPhase->m_tree.GetUserData( id );

			if ( box->Raycast( box->body->GetTransform( ), m_rayCast ) )
			{
				return cb->ReportShape( box );
			}

			return true;
		}

		rpcQueryCallback *cb;
		const rpcBroadPhase *broadPhase;
		rpcRaycastData *m_rayCast;
	};
	
	SceneQueryWrapper wrapper;
	wrapper.m_rayCast = &rayCast;
	wrapper.broadPhase = &m_contactManager.m_broadphase;
	wrapper.cb = cb;
	m_contactManager.m_broadphase.m_tree.Query( &wrapper, rayCast );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::RayCastBatch( rpcQueryCallback *cb, rpcRaycastData* rayCasts, i32 count,
                            const rpcBody* ignore ) const
{
	if ( m_rayStrategy == rpcRayStrategy::Indexed && m_staticRayIndex && m_dynamicRayIndex )
	{
		// Each ray keeps its own closest-hit state, so the per-ray result stays
		// identical to `count` separate RayCast calls; the sweep amortises its
		// population walk across the whole batch.
		m_staticRayIndex->RaycastBatch( cb, rayCasts, count );
		m_dynamicRayIndex->RaycastBatch( cb, rayCasts, count );
		return;
	}

	struct SceneQueryWrapper
	{
		bool TreeCallBack( i32 id, i32 rayIndex )
		{
			rpcBox *box = (rpcBox *)broadPhase->m_tree.GetUserData( id );

			if ( box->Raycast( box->body->GetTransform( ), &rays[ rayIndex ] ) )
			{
				return cb->ReportShape( box, rayIndex, &rays[ rayIndex ] );
			}

			return true;
		}

		rpcQueryCallback *cb;
		const rpcBroadPhase *broadPhase;
		rpcRaycastData *rays;
	};

	SceneQueryWrapper wrapper;
	wrapper.rays = rayCasts;
	wrapper.broadPhase = &m_contactManager.m_broadphase;
	wrapper.cb = cb;
	m_contactManager.m_broadphase.m_tree.QueryBatch( &wrapper, rayCasts, count );
}

//--------------------------------------------------------------------------------------------------
void rpcScene::Dump( FILE* file ) const
{
	fprintf( file, "// Ensure 64/32-bit memory compatability with the dump contents\n" );
	fprintf( file, "assert( sizeof( int* ) == %lu );\n", sizeof( int* ) );
	fprintf( file, "scene.SetGravity( rpcVec3( %.15lf, %.15lf, %.15lf ) );\n", m_gravity.x, m_gravity.y, m_gravity.z );
	fprintf( file, "scene.SetAllowSleep( %s );\n", m_allowSleep ? "true" : "false" );
	fprintf( file, "scene.SetEnableFriction( %s );\n", m_enableFriction ? "true" : "false" );

	fprintf( file, "rpcBody** bodies = (rpcBody**)rpcAlloc( sizeof( rpcBody* ) * %d );\n", m_bodyCount );

	i32 i = 0;
	for ( rpcBody* body = m_bodyList; body; body = body->m_next, ++i )
	{
		body->Dump( file, i );
	}

	fprintf( file, "rpcFree( bodies );\n" );
}
