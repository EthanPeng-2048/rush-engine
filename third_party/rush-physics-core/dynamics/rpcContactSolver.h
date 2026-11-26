//--------------------------------------------------------------------------------------------------
/**
@file	rpcContactSolver.h

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

#ifndef RPCCONTACTSOLVER_H
#define RPCCONTACTSOLVER_H

#include "../math/rpcMath.h"
#include "../common/rpcSettings.h"

//--------------------------------------------------------------------------------------------------
// rpcContactSolver
//--------------------------------------------------------------------------------------------------
struct rpcIsland;
struct rpcVelocityState;

struct rpcContactState
{
	rpcVec3 ra;					// Vector from C.O.M to contact position
	rpcVec3 rb;					// Vector from C.O.M to contact position
	r32 penetration;			// Depth of penetration from collision
	r32 normalImpulse;			// Accumulated normal impulse
	r32 tangentImpulse[ 2 ];	// Accumulated friction impulse
	r32 bias;					// Restitution + baumgarte
	r32 normalMass;				// Normal constraint mass
	r32 tangentMass[ 2 ];		// Tangent constraint mass
	// Index of the manifold point this state came from. Island setup drops
	// points that separated since the last narrowphase (TGS), so state index and
	// manifold index diverge and the impulse write-back needs the mapping.
	i32 srcIndex;
};

struct rpcContactConstraintState
{
	rpcContactState contacts[ 8 ];
	i32 contactCount;
	rpcVec3 tangentVectors[ 2 ];	// Tangent vectors
	rpcVec3 normal;				// From A to B
	rpcVec3 centerA;
	rpcVec3 centerB;
	rpcMat3 iA;
	rpcMat3 iB;
	r32 mA;
	r32 mB;
	r32 restitution;
	r32 friction;
	i32 indexA;
	i32 indexB;
};

struct rpcContactSolver
{
	void Initialize( rpcIsland *island );
	void ShutDown( void );

	void PreSolve( r32 dt );
	void Solve( void );

	rpcIsland *m_island;
	rpcContactConstraintState *m_contacts;
	i32 m_contactCount;
	rpcVelocityState *m_velocities;

	bool m_enableFriction;
};

// The solver walks these arrays per iteration; the size is part of the
// measured per-contact bandwidth budget (see docs/rpc-optimization.md).
static_assert( sizeof( rpcContactState ) == 60, "rpcContactState size changed: re-check solver bandwidth" );

#endif // RPCCONTACTSOLVER_H
