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

#include "rpcScene.h"
#include "../dynamics/rpcBody.h"
#include "../dynamics/rpcIsland.h"
#include "../dynamics/rpcContact.h"
#include "../dynamics/rpcContactSolver.h"
#include "../collision/rpcBox.h"
#include "../common/rpcWorkerPool.h"

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
{
}

//--------------------------------------------------------------------------------------------------
rpcScene::~rpcScene( )
{
	Shutdown( );
	// m_workers is the process-wide shared singleton (rpcWorkerPool::Shared)
	// — it outlives every scene and must never be deleted here.
}

//--------------------------------------------------------------------------------------------------
void rpcScene::Step( )
{
	if ( m_newBox )
	{
		m_contactManager.m_broadphase.UpdatePairs( );
		m_newBox = false;
	}

	m_contactManager.TestCollisions( );

	SolveIslands( );
	SyncAndFind( );
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
	// Dynamic bodies only: static eIsland flags are reset right after each
	// island build (shared statics must re-join later islands), so they
	// never leak across steps — clearing them here would walk 6000+ track
	// bodies per step for nothing.
	for ( rpcBody* body = m_bodyList; body; body = body->m_next )
	{
		if ( !( body->m_flags & rpcBody::eStatic ) )
			body->m_flags &= ~rpcBody::eIsland;
	}

	// Worst case: every DYNAMIC body seeds its own island — the static
	// track bodies (thousands) must never size the batch pools, or the pools
	// balloon into hundreds of megabytes and TLB-trash every step. Pools
	// grow ×2 and never shrink; each island spans a contiguous slice written
	// through span views, so steady-state solving allocates and clears
	// nothing.
	const size_t maxIslands = (size_t)rpcMax( 1, m_dynamicCount );
	// Island body capacity: an island can hold every dynamic body plus the
	// static bodies it touches (a static body appears in many islands), so
	// cap at the full scene size — still cheap once maxIslands is small.
	const size_t islandCap = (size_t)m_bodyCount;
	// Contact pools size against the monotone contact peak, never the raw
	// per-step count: the count jitters (240/334/274...) and a pool that
	// reallocates on every step burns tens of microseconds of copies.
	m_contactPeak = rpcMax( m_contactPeak, m_contactManager.m_contactCount );
	const size_t contactCap = (size_t)rpcMax( 1, m_contactPeak );
	const auto growPool = []( auto& pool, size_t need ) {
		if ( pool.capacity( ) < need )
			pool.reserve( need * 2 );
	};
	growPool( m_poolBodies, maxIslands * islandCap );
	growPool( m_poolVelocities, maxIslands * islandCap );
	growPool( m_poolContacts, maxIslands * contactCap );
	growPool( m_poolContactStates, maxIslands * contactCap );
	if ( m_dfsStack.size( ) < islandCap )
		m_dfsStack.resize( islandCap );
	if ( m_islands.size( ) < maxIslands )
		m_islands.resize( maxIslands );

	// Build every island and Initialize() right away: Initialize snapshots
	// body->m_islandIndex, which a shared static body would overwrite if the
	// islands were built first and initialized later.
	i32 islandCount = 0;
	for ( rpcBody* seed = m_bodyList; seed; seed = seed->m_next )
	{
		// Seed cannot be apart of an island already
		if ( seed->m_flags & rpcBody::eIsland )
			continue;

		// Seed must be awake
		if ( !(seed->m_flags & rpcBody::eAwake) )
			continue;

		// Seed cannot be a static body in order to keep islands
		// as small as possible
		if ( seed->m_flags & rpcBody::eStatic )
			continue;

		const size_t slot = (size_t)islandCount;
		rpcIsland& island = m_islands[ slot ];
		island.Reset(
			std::span<rpcBody*>( m_poolBodies.data( ) + slot * islandCap, islandCap ),
			std::span<rpcVelocityState>( m_poolVelocities.data( ) + slot * islandCap, islandCap ),
			std::span<rpcContactConstraint*>( m_poolContacts.data( ) + slot * contactCap, contactCap ),
			std::span<rpcContactConstraintState>( m_poolContactStates.data( ) + slot * contactCap, contactCap ),
			m_dt, m_gravity, m_iterations, m_allowSleep, m_enableFriction );

		BuildIsland( seed, island, m_dfsStack.data( ), (i32)m_dfsStack.size( ) );

		// Clear the static-body island flags right away: a shared static body
		// (ground, walls) must be re-joinable by later islands, otherwise its
		// m_islandIndex stays stale and Solve() indexes out of bounds. Keep
		// Initialize() before the next island overwrites those indices.
		for ( i32 i = 0; i < island.m_bodyCount; ++i )
		{
			rpcBody *body = island.m_bodies[ i ];
			if ( body->m_flags & rpcBody::eStatic )
				body->m_flags &= ~rpcBody::eIsland;
		}
		island.Initialize( );

		++islandCount;
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
		return;
	}
	const i32 workers = (i32)m_workers->WorkerCount( ) + 1;
	if ( islandCount < 4 * workers )
	{
		for ( i32 i = 0; i < islandCount; ++i )
			m_islands[ (size_t)i ].Solve( );
		return;
	}
	m_workers->Run( rpcSolveIslandTask, this, islandCount );
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
	// Update the broadphase AABBs
	for ( rpcBody* body = m_bodyList; body; body = body->m_next )
	{
		if ( body->m_flags & rpcBody::eStatic )
			continue;

		body->SynchronizeProxies( );
	}

	// Look for new contacts
	m_contactManager.FindNewContacts( );

	// Clear all forces — dynamic bodies only: static bodies never receive
	// forces (L2 applies forces to vehicles), so skipping them removes a
	// 6000+ body walk from every step.
	for ( rpcBody* body = m_bodyList; body; body = body->m_next )
	{
		if ( body->m_flags & rpcBody::eStatic )
			continue;
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
		++m_dynamicCount;

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
		--m_dynamicCount;

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
void rpcScene::RayCast( rpcQueryCallback *cb, rpcRaycastData& rayCast ) const
{
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
