//--------------------------------------------------------------------------------------------------
/**
@file	rpcContactManager.h

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

#ifndef RPCCONTACTMANAGER_H
#define RPCCONTACTMANAGER_H

#include "../common/rpcTypes.h"
#include "../broadphase/rpcBroadPhase.h"
#include "../common/rpcMemory.h"
#include "../math/rpcVec3.h"
#include "rpcPairMap.h"

#include <vector>

//--------------------------------------------------------------------------------------------------
// rpcContactManager
//--------------------------------------------------------------------------------------------------
struct rpcContactConstraint;
struct rpcManifold;
class rpcContactListener;
struct rpcBox;
class rpcBody;
class rpcRender;
class rpcStack;

class rpcContactManager
{
public:
	rpcContactManager( rpcStack* stack );

	// Add a new contact constraint for a pair of objects
	// unless the contact constraint already exists
	void AddContact( rpcBox *A, rpcBox *B );

	// New-constraint budget per step (0 = unlimited). Creating a constraint
	// allocates and initialises a manifold, so a pile-up that suddenly exposes
	// thousands of new pairs would otherwise spike a single step; the overflow
	// is queued and drained by the following steps.
	void SetNewContactBudget( i32 perStep ) { m_newContactBudget = perStep; }
	i32 NewContactsThisStep( ) const { return m_newContactsThisStep; }
	i32 DeferredPairs( ) const { return (i32)m_deferred.size( ); }

	// Has broadphase find all contacts and call AddContact on the
	// ContactManager for each pair found
	void FindNewContacts( void );

	// Remove a specific contact
	void RemoveContact( rpcContactConstraint *contact );

	// Remove all contacts from a body
	void RemoveContactsFromBody( rpcBody *body );
	void RemoveFromBroadphase( rpcBody *body );

	// Remove contacts without broadphase overlap
	// Solves contact manifolds
	void TestCollisions( void );

	// Every-substep companion to decimated narrowphase (TGS): drop constraints
	// whose stored local anchors have opened up. Without this, contacts that
	// separated between two detection passes keep bridging bodies in the island
	// graph, and a pile-up collapses into one giant serial island (measured: 35
	// islands / 8.5 k contacts -> 1 island / 10 k contacts, 4x the step cost).
	void PruneSeparatedContacts( );

	void RenderContacts( rpcRender* debugDrawer ) const;

private:
	// Warm-start carry-over from the previous step: only the fields warm-start
	// matching needs, so TestCollisions never copies a full rpcManifold.
	struct WarmStartSnapshot
	{
		rpcVec3 tangent[ 2 ];
		i32 count;
		struct
		{
			i32 fpKey;
			r32 normalImpulse;
			r32 tangentImpulse[ 2 ];
		} pts[ 8 ];
	};

	// TestCollisions pipeline steps (one concern each):
	bool ContactStillValid( const rpcContactConstraint* c ) const;	// CanCollide + broadphase overlap
	void NarrowphaseUpdate( rpcContactConstraint* c );				// SolveCollision + ComputeBasis
	void WarmStartMatch( rpcContactConstraint* c, const WarmStartSnapshot& old );	// feature-pair match + friction re-projection
	void FireContactEdges( rpcContactConstraint* c );				// Begin/End edges -> listener

	static void CaptureWarmStart( const rpcManifold& m, WarmStartSnapshot& out );

	// Consume queued pairs (bounded by the per-step budget) before the
	// broadphase offers this step's candidates.
	void DrainDeferred( );
	void CreateConstraint( rpcBox* A, rpcBox* B );

	rpcContactConstraint* m_contactList;
	i32 m_contactCount;
	// O(1) persistence lookup: a shape pair maps to its live constraint, and a
	// body pair maps to how many constraints it already owns (the pile-up cap).
	rpcPairMap<rpcContactConstraint*> m_pairMap;
	rpcPairMap<i32> m_bodyPairCounts;
	struct DeferredPair
	{
		rpcBox* A;
		rpcBox* B;
	};
	std::vector<DeferredPair> m_deferred;
	i32 m_newContactBudget;
	i32 m_newContactsThisStep;
	rpcStack* m_stack;
	rpcPagedAllocator m_allocator;
	rpcBroadPhase m_broadphase;
	rpcContactListener *m_contactListener;

	friend class rpcBroadPhase;
	friend class rpcScene;
	friend struct rpcBox;
	friend class rpcBody;
};

#endif // RPCCONTACTMANAGER_H
