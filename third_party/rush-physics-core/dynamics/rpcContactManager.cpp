//--------------------------------------------------------------------------------------------------
/**
@file	rpcContactManager.cpp

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

#include "rpcContactManager.h"
#include "../collision/rpcBox.h"
#include "rpcBody.h"
#include "rpcContact.h"
#include "../scene/rpcScene.h"
#include "../debug/rpcRender.h"

//--------------------------------------------------------------------------------------------------
// rpcContactManager
//--------------------------------------------------------------------------------------------------
rpcContactManager::rpcContactManager( rpcStack* stack )
	: m_stack( stack )
	, m_allocator( sizeof( rpcContactConstraint ), 256 )
	, m_broadphase( this )
{
	m_contactList = NULL;
	m_contactCount = 0;
	m_contactListener = NULL;
	m_newContactBudget = 0;
	m_newContactsThisStep = 0;
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::AddContact( rpcBox *A, rpcBox *B )
{
	rpcBody *bodyA = A->body;
	rpcBody *bodyB = B->body;
	if ( !bodyA->CanCollide( bodyB ) )
		return;

	// O(1) "does this shape pair already own a constraint?". The qu3e original
	// walked bodyA's contact-edge list, i.e. O(contacts of that body) — in a
	// pile-up one body carries hundreds of edges and the broadphase offers tens
	// of thousands of candidates per step, so the lookup, not the collision
	// maths, was the frame's biggest cost (17.9 ms of a 20.3 ms worst-step
	// broadphase on 120 cars). See rpcPairMap.h.
	const u64 shapeKey = rpcPairMap<rpcContactConstraint*>::PairKey( A->uid, B->uid );
	if ( m_pairMap.Find( shapeKey ) )
		return;

	// Dynamic pairs keep a hard cap per body pair: a pile-up of composite bodies
	// must not multiply manifolds (11 shapes each => up to 121 per pair).
	const bool capPair = !( bodyA->m_flags & rpcBody::eStatic ) &&
	                     !( bodyB->m_flags & rpcBody::eStatic );
	const u64 bodyKey = rpcPairMap<i32>::PairKey( bodyA->m_uid, bodyB->m_uid );
	if ( capPair )
	{
		const i32* count = m_bodyPairCounts.Find( bodyKey );
		if ( count && *count >= RPC_MAX_CONTACTS_PER_DYNAMIC_PAIR )
			return;
	}

	// Work budget: queue the overflow instead of spiking this step.
	if ( m_newContactBudget > 0 && m_newContactsThisStep >= m_newContactBudget )
	{
		m_deferred.push_back( { A, B } );
		return;
	}

	CreateConstraint( A, B );

	++m_newContactsThisStep;
	if ( capPair )
	{
		i32* count = m_bodyPairCounts.Find( bodyKey );
		if ( count )
			++*count;
		else
			m_bodyPairCounts.Insert( bodyKey, 1 );
	}
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::DrainDeferred( )
{
	if ( m_deferred.empty( ) )
		return;

	// Pairs held over from earlier steps: the shapes may have separated while
	// queued, and the pair may even have been created by another candidate in
	// the meantime (AddContact's map check covers that).
	i32 keep = 0;
	const i32 limit = m_newContactBudget > 0 ? m_newContactBudget : 0x7fffffff;
	for ( size_t i = 0; i < m_deferred.size( ); ++i )
	{
		const DeferredPair& pair = m_deferred[ i ];
		if ( m_newContactsThisStep >= limit )
		{
			m_deferred[ (size_t)keep++ ] = pair;
			continue;
		}

		if ( !m_broadphase.TestOverlap( pair.A->broadPhaseIndex, pair.B->broadPhaseIndex ) )
			continue;

		AddContact( pair.A, pair.B );
	}

	m_deferred.resize( (size_t)keep );
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::CreateConstraint( rpcBox* A, rpcBox* B )
{
	rpcBody *bodyA = A->body;
	rpcBody *bodyB = B->body;

	// Create new contact
	rpcContactConstraint *contact = (rpcContactConstraint*)m_allocator.Allocate( );
	contact->A = A;
	contact->B = B;
	contact->bodyA = A->body;
	contact->bodyB = B->body;
	contact->manifold.SetPair( A, B );
	contact->m_flags = 0;
	contact->friction = rpcMixFriction( A, B );
	contact->restitution = rpcMixRestitution( A, B );
	contact->manifold.contactCount = 0;

	for ( i32 i = 0; i < 8; ++i )
		contact->manifold.contacts[ i ].warmStarted = 0;

	contact->prev = NULL;
	contact->next = m_contactList;
	if ( m_contactList )
		m_contactList->prev = contact;
	m_contactList = contact;

	// Connect A
	contact->edgeA.constraint = contact;
	contact->edgeA.other = bodyB;

	contact->edgeA.prev = NULL;
	contact->edgeA.next = bodyA->m_contactList;
	if ( bodyA->m_contactList )
		bodyA->m_contactList->prev = &contact->edgeA;
	bodyA->m_contactList = &contact->edgeA;

	// Connect B
	contact->edgeB.constraint = contact;
	contact->edgeB.other = bodyA;

	contact->edgeB.prev = NULL;
	contact->edgeB.next = bodyB->m_contactList;
	if ( bodyB->m_contactList )
		bodyB->m_contactList->prev = &contact->edgeB;
	bodyB->m_contactList = &contact->edgeB;

	bodyA->SetToAwake( );
	bodyB->SetToAwake( );

	++m_contactCount;

	// The map stores the constraint so RemoveContact can find it by key; the
	// pointer is filled in here, after the value slot exists.
	m_pairMap.Insert( rpcPairMap<rpcContactConstraint*>::PairKey( A->uid, B->uid ), contact );
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::FindNewContacts( )
{
	// O(1) lookups mean the per-step dedupe walk is gone; only the raw counters
	// are reset here. Queued pairs go first so an overflow from a previous step
	// cannot be starved by a fresh burst.
	m_newContactsThisStep = 0;
	DrainDeferred( );
	m_broadphase.UpdatePairs( );
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::RemoveContact( rpcContactConstraint *contact )
{
	rpcBody *A = contact->bodyA;
	rpcBody *B = contact->bodyB;

	// Remove from A
	if ( contact->edgeA.prev )
		contact->edgeA.prev->next = contact->edgeA.next;

	if ( contact->edgeA.next )
		contact->edgeA.next->prev = contact->edgeA.prev;

	if ( &contact->edgeA == A->m_contactList )
		A->m_contactList = contact->edgeA.next;

	// Remove from B
	if ( contact->edgeB.prev )
		contact->edgeB.prev->next = contact->edgeB.next;

	if ( contact->edgeB.next )
		contact->edgeB.next->prev = contact->edgeB.prev;

	if ( &contact->edgeB == B->m_contactList )
		B->m_contactList = contact->edgeB.next;

	A->SetToAwake( );
	B->SetToAwake( );

	// Remove contact from the manager
	if ( contact->prev )
		contact->prev->next = contact->next;

	if ( contact->next )
		contact->next->prev = contact->prev;

	if ( contact == m_contactList )
		m_contactList = contact->next;

	--m_contactCount;

	// Drop the persistence-lookup entry and give the body pair its budget back.
	m_pairMap.Erase( rpcPairMap<rpcContactConstraint*>::PairKey( contact->A->uid, contact->B->uid ) );
	if ( !( contact->bodyA->m_flags & rpcBody::eStatic ) &&
	     !( contact->bodyB->m_flags & rpcBody::eStatic ) )
	{
		const u64 bodyKey = rpcPairMap<i32>::PairKey( contact->bodyA->m_uid, contact->bodyB->m_uid );
		if ( i32* count = m_bodyPairCounts.Find( bodyKey ) )
		{
			if ( --*count <= 0 )
				m_bodyPairCounts.Erase( bodyKey );
		}
	}

	m_allocator.Free( contact );
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::RemoveContactsFromBody( rpcBody *body )
{
	// Queued pairs hold raw shape pointers and the body is going away: drop the
	// queue rather than let a later drain touch freed shapes. Body destruction
	// only happens on a race rebuild, so losing the queue costs nothing.
	if ( !m_deferred.empty( ) )
		m_deferred.clear( );

	rpcContactEdge* edge = body->m_contactList;

	while( edge )
	{
		rpcContactEdge* next = edge->next;
		RemoveContact( edge->constraint );
		edge = next;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::RemoveFromBroadphase( rpcBody *body )
{
	rpcBox* box = body->m_boxes;

	while ( box )
	{
		m_broadphase.RemoveBox( box );
		box = box->next;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::PruneSeparatedContacts( void )
{
	const r32 slop = -RPC_PENETRATION_SLOP;

	rpcContactConstraint* constraint = m_contactList;
	while ( constraint )
	{
		rpcContactConstraint* next = constraint->next;

		// A constraint created since the last detection pass has no manifold yet:
		// it is not stale, it is simply untested — keep it.
		if ( constraint->manifold.contactCount == 0 )
		{
			constraint = next;
			continue;
		}

		const rpcMat3& rotA = constraint->bodyA->m_tx.rotation;
		const rpcMat3& rotB = constraint->bodyB->m_tx.rotation;

		bool touching = false;
		for ( i32 i = 0; i < constraint->manifold.contactCount; ++i )
		{
			const rpcContact& cp = constraint->manifold.contacts[ i ];
			const rpcVec3 normal = rpcMul( rotA, cp.localNormalA );
			const rpcVec3 drift = rpcMul( rotA, cp.localAnchorA ) - rpcMul( rotB, cp.localAnchorB );

			if ( cp.penetration + rpcDot( drift, normal ) >= slop )
			{
				touching = true;
				break;
			}
		}

		if ( !touching )
		{
			if ( m_contactListener && ( constraint->m_flags & rpcContactConstraint::eColliding ) )
				m_contactListener->EndContact( constraint );
			RemoveContact( constraint );
		}

		constraint = next;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::CaptureWarmStart( const rpcManifold& m, WarmStartSnapshot& out )
{
	out.tangent[ 0 ] = m.tangentVectors[ 0 ];
	out.tangent[ 1 ] = m.tangentVectors[ 1 ];
	out.count = m.contactCount;

	for ( i32 i = 0; i < m.contactCount; ++i )
	{
		out.pts[ i ].fpKey = m.contacts[ i ].fp.key;
		out.pts[ i ].normalImpulse = m.contacts[ i ].normalImpulse;
		out.pts[ i ].tangentImpulse[ 0 ] = m.contacts[ i ].tangentImpulse[ 0 ];
		out.pts[ i ].tangentImpulse[ 1 ] = m.contacts[ i ].tangentImpulse[ 1 ];
	}
}

//--------------------------------------------------------------------------------------------------
bool rpcContactManager::ContactStillValid( const rpcContactConstraint* c ) const
{
	if ( !c->bodyA->CanCollide( c->bodyB ) )
		return false;

	return m_broadphase.TestOverlap( c->A->broadPhaseIndex, c->B->broadPhaseIndex );
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::NarrowphaseUpdate( rpcContactConstraint* c )
{
	c->SolveCollision( );
	rpcManifold* m = &c->manifold;
	rpcComputeBasis( m->normal, m->tangentVectors, m->tangentVectors + 1 );
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::WarmStartMatch( rpcContactConstraint* c, const WarmStartSnapshot& old )
{
	rpcManifold* manifold = &c->manifold;

	for ( i32 i = 0; i < manifold->contactCount; ++i )
	{
		rpcContact *pc = manifold->contacts + i;
		pc->tangentImpulse[ 0 ] = pc->tangentImpulse[ 1 ] = pc->normalImpulse = r32( 0.0 );
		u8 oldWarmStart = pc->warmStarted;
		pc->warmStarted = u8( 0 );

		for ( i32 j = 0; j < old.count; ++j )
		{
			if ( pc->fp.key == old.pts[ j ].fpKey )
			{
				pc->normalImpulse = old.pts[ j ].normalImpulse;

				// Attempt to re-project old friction solutions
				rpcVec3 friction = old.tangent[ 0 ] * old.pts[ j ].tangentImpulse[ 0 ]
					+ old.tangent[ 1 ] * old.pts[ j ].tangentImpulse[ 1 ];
				pc->tangentImpulse[ 0 ] = rpcDot( friction, manifold->tangentVectors[ 0 ] );
				pc->tangentImpulse[ 1 ] = rpcDot( friction, manifold->tangentVectors[ 1 ] );
				pc->warmStarted = rpcMax( oldWarmStart, u8( oldWarmStart + 1 ) );
				break;
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::FireContactEdges( rpcContactConstraint* c )
{
	if ( !m_contactListener )
		return;

	i32 now_colliding = c->m_flags & rpcContactConstraint::eColliding;
	i32 was_colliding = c->m_flags & rpcContactConstraint::eWasColliding;

	if ( now_colliding && !was_colliding )
		m_contactListener->BeginContact( c );

	else if ( !now_colliding && was_colliding )
		m_contactListener->EndContact( c );
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::TestCollisions( void )
{
	rpcContactConstraint* constraint = m_contactList;

	while( constraint )
	{
		constraint->m_flags &= ~rpcContactConstraint::eIsland;

		if( !constraint->bodyA->IsAwake( ) && !constraint->bodyB->IsAwake( ) )
		{
			constraint = constraint->next;
			continue;
		}

		if ( !ContactStillValid( constraint ) )
		{
			rpcContactConstraint* next = constraint->next;
			RemoveContact( constraint );
			constraint = next;
			continue;
		}

		WarmStartSnapshot old;
		CaptureWarmStart( constraint->manifold, old );
		NarrowphaseUpdate( constraint );
		WarmStartMatch( constraint, old );
		FireContactEdges( constraint );

		constraint = constraint->next;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcContactManager::RenderContacts( rpcRender* render ) const
{
	const rpcContactConstraint *contact = m_contactList;

	while ( contact )
	{
		const rpcManifold *m = &contact->manifold;

		if ( !(contact->m_flags & rpcContactConstraint::eColliding) )
		{
			contact = contact->next;
			continue;
		}

		for ( i32 j = 0; j < m->contactCount; ++j)
		{
			const rpcContact *c = m->contacts + j;
			f32 blue = (f32)(255 - c->warmStarted) / 255.0f;
			f32 red = 1.0f - blue;
			render->SetScale( 10.0f, 10.0f, 10.0f );
			render->SetPenColor( red, blue, blue );
			render->SetPenPosition( c->position.x, c->position.y, c->position.z );
			render->Point( );

			if ( m->A->body->IsAwake( ) )
				render->SetPenColor( 1.0f, 1.0f, 1.0f );
			else
				render->SetPenColor( 0.2f, 0.2f, 0.2f );

			render->SetPenPosition( c->position.x, c->position.y, c->position.z );
			render->Line(
				c->position.x + m->normal.x * 0.5f,
				c->position.y + m->normal.y * 0.5f,
				c->position.z + m->normal.z * 0.5f
				);
		}

		contact = contact->next;
	}

	render->SetScale( 1.0f, 1.0f, 1.0f );
}
