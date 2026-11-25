//--------------------------------------------------------------------------------------------------
/**
@file	rpcGeometry.h

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

#ifndef RPCGEOMETRY_H
#define RPCGEOMETRY_H

#include "../math/rpcMath.h"

//--------------------------------------------------------------------------------------------------
// rpcAABB
//--------------------------------------------------------------------------------------------------
struct rpcAABB
{
	rpcVec3 min;
	rpcVec3 max;

	bool Contains( const rpcAABB& other ) const;
	bool Contains( const rpcVec3& point ) const;
	r32 SurfaceArea( ) const;
};

// Layout contract: the broadphase node, the ray indexes and the per-body proxy
// all pack AABBs; growing this silently blows the cache budget of every query.
static_assert( sizeof( rpcAABB ) == 24, "rpcAABB must stay two rpcVec3 (24 B)" );

const rpcAABB rpcCombine( const rpcAABB& a, const rpcAABB& b );

// Grow by RPC_AABB_MARGIN on every axis. The single definition of the broadphase
// "fat" AABB: the tree, the ray indexes and the box proxies all use it, so a
// shape is rejected by exactly the same bounds whichever strategy is active.
rpcAABB rpcFatten( const rpcAABB& a );

// In-place union (used to maintain per-body proxy bounds).
void rpcUnion( rpcAABB& out, const rpcAABB& in );

// True when the bounds contain no point (freshly reset accumulator).
bool rpcIsEmpty( const rpcAABB& a );

//--------------------------------------------------------------------------------------------------
// rpcHalfSpace
//--------------------------------------------------------------------------------------------------
struct rpcHalfSpace
{
	rpcHalfSpace( );
	rpcHalfSpace( const rpcVec3& normal, r32 distance );

	void Set( const rpcVec3& a, const rpcVec3& b, const rpcVec3& c );
	void Set( const rpcVec3& n, const rpcVec3& p );
	const rpcVec3 Origin( ) const;
	r32 Distance( const rpcVec3& p ) const;
	const rpcVec3 Projected( const rpcVec3& p ) const;

	rpcVec3 normal;
	r32 distance;
};

//--------------------------------------------------------------------------------------------------
// rpcRaycastData
//--------------------------------------------------------------------------------------------------
struct rpcRaycastData
{
	rpcVec3 start;	// Beginning point of the ray
	rpcVec3 dir;		// Direction of the ray (normalized)
	r32 t;			// Time specifying ray endpoint

	r32 toi;		// Solved time of impact
	rpcVec3 normal;	// Surface normal at impact

	void Set( const rpcVec3& startPoint, const rpcVec3& direction, r32 endPointTime );

	// Uses toi, start and dir to compute the point at toi. Should
	// only be called after a raycast has been conducted with a
	// return value of true.
	const rpcVec3 GetImpactPoint( ) const;
};

#include "rpcGeometry.inl"

#endif // RPCGEOMETRY_H
