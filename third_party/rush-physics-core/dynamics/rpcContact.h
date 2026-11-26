//--------------------------------------------------------------------------------------------------
/**
@file	rpcContact.h

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

#ifndef RPCCONTACT_H
#define RPCCONTACT_H

#include "../math/rpcMath.h"
#include "../common/rpcSettings.h"
#include "../collision/rpcBox.h"
#include "../collision/rpcCollide.h"

//--------------------------------------------------------------------------------------------------
// rpcContact
//--------------------------------------------------------------------------------------------------
class rpcBody;
struct rpcBox;
struct rpcContactConstraint;

// Restitution mixing. The idea is to use the maximum bounciness, so bouncy
// objects will never not bounce during collisions.
inline r32 rpcMixRestitution( const rpcBox* A, const rpcBox* B )
{
	return rpcMax( A->restitution, B->restitution );
}

// Friction mixing. The idea is to allow a very low friction value to
// drive down the mixing result. Example: anything slides on ice.
inline r32 rpcMixFriction( const rpcBox* A, const rpcBox* B )
{
	return std::sqrt( A->friction * B->friction );
}

// in stands for "incoming"
// out stands for "outgoing"
// I stands for "incident"
// R stands for "reference"
// See D. Gregorius GDC 2015 on creating contacts for more details
// Each feature pair is used to cache solutions from one physics tick to another. This is
// called warmstarting, and lets boxes stack and stay stable. Feature pairs identify points
// of contact over multiple physics ticks. Each feature pair is the junction of an incoming
// feature and an outgoing feature, usually a result of clipping routines. The exact info
// stored in the feature pair can be arbitrary as long as the result is a unique ID for a
// given intersecting configuration.
union rpcFeaturePair
{
	struct
	{
		u8 inR;
		u8 outR;
		u8 inI;
		u8 outI;
	};

	i32 key;
};

struct rpcContact
{
	rpcVec3 position;			// World coordinate of contact
	r32 penetration;			// Depth of penetration from collision
	r32 normalImpulse;			// Accumulated normal impulse
	r32 tangentImpulse[ 2 ];	// Accumulated friction impulse
	rpcFeaturePair fp;			// Features on A and B for this contact
	u8 warmStarted;				// Used for debug rendering

	// Narrowphase runs once per frame while the solver runs every substep (TGS,
	// see rpcSceneConfig::contactRefreshSubsteps), so a world-space contact
	// point would be stale by up to a frame of motion. These anchors are the
	// narrowphase result expressed in each body's frame: the solver re-projects
	// them every substep (rpcIsland::Initialize) and derives penetration from
	// how far they have moved apart since.
	rpcVec3 localAnchorA;		// Contact point in A's frame (relative to its COM)
	rpcVec3 localAnchorB;		// Contact point in B's frame
	rpcVec3 localNormalA;		// Manifold normal (A -> B) in A's frame
};

struct rpcManifold
{
	void SetPair( rpcBox *a, rpcBox *b );

	rpcBox *A;
	rpcBox *B;

	rpcVec3 normal;				// From A to B
	rpcVec3 tangentVectors[ 2 ];	// Tangent vectors
	rpcContact contacts[ 8 ];
	i32 contactCount;

	rpcManifold* next;
	rpcManifold* prev;

	bool sensor;
};

struct rpcContactEdge
{
	rpcBody *other;
	rpcContactConstraint *constraint;
	rpcContactEdge* next;
	rpcContactEdge* prev;
};

struct rpcContactConstraint
{
	void SolveCollision( void );

	rpcBox *A, *B;
	rpcBody *bodyA, *bodyB;

	rpcContactEdge edgeA;
	rpcContactEdge edgeB;
	rpcContactConstraint* next;
	rpcContactConstraint* prev;

	r32 friction;
	r32 restitution;

	rpcManifold manifold;

	enum
	{
		eColliding    = 0x00000001, // Set when contact collides during a step
		eWasColliding = 0x00000002, // Set when two objects stop colliding
		eIsland       = 0x00000004, // For internal marking during island forming
	};

	i32 m_flags;

	friend class rpcContactManager;
	friend class rpcScene;
	friend struct rpcIsland;
	friend struct rpcContactSolver;
};

#endif // RPCCONTACT_H
