//--------------------------------------------------------------------------------------------------
/**
@file	rpcIsland.h

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

#ifndef RPCISLAND_H
#define RPCISLAND_H

#include "../math/rpcMath.h"
#include "../common/rpcGeometry.h"
#include "../common/rpcSettings.h"

//--------------------------------------------------------------------------------------------------
// rpcIsland
//--------------------------------------------------------------------------------------------------
class rpcBroadPhase;
class rpcBody;
struct rpcContactConstraint;
struct rpcContactConstraintState;

struct rpcVelocityState
{
	rpcVec3 w;
	rpcVec3 v;
};

struct rpcIsland
{
	void Solve( );
	void Add( rpcBody *body );
	void Add( rpcContactConstraint *contact );
	void Initialize( );

	rpcBody **m_bodies;
	rpcVelocityState *m_velocities;
	i32 m_bodyCapacity;
	i32 m_bodyCount;

	rpcContactConstraint **m_contacts;
	rpcContactConstraintState *m_contactStates;
	i32 m_contactCount;
	i32 m_contactCapacity;

	r32 m_dt;
	rpcVec3 m_gravity;
	i32 m_iterations;

	bool m_allowSleep;
	bool m_enableFriction;
};

#endif // RPCISLAND_H
