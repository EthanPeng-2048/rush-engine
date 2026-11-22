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

#include "rpcScene.h"
#include "../dynamics/rpcBody.h"
#include "../dynamics/rpcIsland.h"
#include "../dynamics/rpcContact.h"
#include "../dynamics/rpcIsland.h"
#include "../dynamics/rpcContactSolver.h"
#include "../collision/rpcBox.h"

//--------------------------------------------------------------------------------------------------
// rpcScene
//--------------------------------------------------------------------------------------------------
rpcScene::rpcScene( r32 dt, const rpcVec3& gravity, i32 iterations )
	: m_contactManager( &m_stack )
	, m_boxAllocator( sizeof( rpcBox ), 256 )
	, m_bodyCount( 0 )
	, m_bodyList( NULL )
	, m_gravity( gravity )
	, m_dt( dt )
	, m_iterations( iterations )
	, m_newBox( false )
	, m_allowSleep( true )
	, m_enableFriction( true )
{
}

//--------------------------------------------------------------------------------------------------
rpcScene::~rpcScene( )
{
	Shutdown( );
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

	for ( rpcBody* body = m_bodyList; body; body = body->m_next )
		body->m_flags &= ~rpcBody::eIsland;

	// Size the stack island, pick worst case size
	m_stack.Reserve(
		sizeof( rpcBody* ) * m_bodyCount
		+ sizeof( rpcVelocityState ) * m_bodyCount
		+ sizeof( rpcContactConstraint* ) * m_contactManager.m_contactCount
		+ sizeof( rpcContactConstraintState ) * m_contactManager.m_contactCount
		+ sizeof( rpcBody* ) * m_bodyCount
	);

	rpcIsland island;
	island.m_bodyCapacity = m_bodyCount;
	island.m_contactCapacity = m_contactManager.m_contactCount;
	island.m_bodies = (rpcBody**)m_stack.Allocate( sizeof( rpcBody* ) * m_bodyCount );
	island.m_velocities = (rpcVelocityState *)m_stack.Allocate( sizeof( rpcVelocityState ) * m_bodyCount );
	island.m_contacts = (rpcContactConstraint **)m_stack.Allocate( sizeof( rpcContactConstraint* ) * island.m_contactCapacity );
	island.m_contactStates = (rpcContactConstraintState *)m_stack.Allocate( sizeof( rpcContactConstraintState ) * island.m_contactCapacity );
	island.m_allowSleep = m_allowSleep;
	island.m_enableFriction = m_enableFriction;
	island.m_bodyCount = 0;
	island.m_contactCount = 0;
	island.m_dt = m_dt;
	island.m_gravity = m_gravity;
	island.m_iterations = m_iterations;

	// Build each active island and then solve each built island
	i32 stackSize = m_bodyCount;
	rpcBody** stack = (rpcBody**)m_stack.Allocate( sizeof( rpcBody* ) * stackSize );
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

		i32 stackCount = 0;
		stack[ stackCount++ ] = seed;
		island.m_bodyCount = 0;
		island.m_contactCount = 0;

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

		assert( island.m_bodyCount != 0 );

		island.Initialize( );
		island.Solve( );

		// Reset all static island flags
		// This allows static bodies to participate in other island formations
		for ( i32 i = 0; i < island.m_bodyCount; i++ )
		{
			rpcBody *body = island.m_bodies[ i ];

			if ( body->m_flags & rpcBody::eStatic )
				body->m_flags &= ~rpcBody::eIsland;
		}
	}

	m_stack.Free( stack );
	m_stack.Free( island.m_contactStates );
	m_stack.Free( island.m_contacts );
	m_stack.Free( island.m_velocities );
	m_stack.Free( island.m_bodies );

	// Update the broadphase AABBs
	for ( rpcBody* body = m_bodyList; body; body = body->m_next )
	{
		if ( body->m_flags & rpcBody::eStatic )
			continue;

		body->SynchronizeProxies( );
	}

	// Look for new contacts
	m_contactManager.FindNewContacts( );

	// Clear all forces
	for ( rpcBody* body = m_bodyList; body; body = body->m_next )
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
