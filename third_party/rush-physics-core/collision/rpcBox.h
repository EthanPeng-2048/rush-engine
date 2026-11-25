//--------------------------------------------------------------------------------------------------
/**
@file	rpcBox.h

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

#ifndef RPCBOX_H
#define RPCBOX_H

#include "../math/rpcVec3.h"
#include "../math/rpcMat3.h"
#include "../math/rpcTransform.h"
#include "../debug/rpcRender.h"

//--------------------------------------------------------------------------------------------------
// rpcMassData
//--------------------------------------------------------------------------------------------------
struct rpcMassData
{
	rpcMat3 inertia;
	rpcVec3 center;
	r32 mass;
};

//--------------------------------------------------------------------------------------------------
// rpcBox
//--------------------------------------------------------------------------------------------------
struct rpcBox
{
	rpcTransform local;
	rpcVec3 e; // extent, as in the extent of each OBB axis

	rpcBox* next;
	class rpcBody* body;
	r32 friction;
	r32 restitution;
	r32 density;
	i32 broadPhaseIndex;
	// Creation-order id (per scene, monotonic). Closest-hit selection breaks
	// exact distance ties by it, so every ray strategy returns the same shape.
	i32 uid;
	// Leaf id inside the dynamic ray tree (rpcDynamicTreeRayIndex), -1 = not in it.
	i32 rayIndexId;
	mutable void* userData;
	mutable bool sensor;

	// Fattened world AABB, maintained by the owner (body/scene) on insert and
	// on every move past the margin. Ray indexes and narrowphase pruning read
	// it instead of recomputing an OBB AABB per query.
	rpcAABB fatAABB;

	void SetUserdata( void* data ) const;
	void* GetUserdata( ) const;

	// True when `local` carries no rotation: lets the hot world-transform path
	// skip a 3x3 matrix multiply. Derived on demand — never cached, because
	// `local` is also written directly by callers and by unit tests.
	bool HasLocalRotation( ) const;
	void SetSensor( bool isSensor );

	// body transform (tx) composed with the shape's local transform.
	rpcTransform WorldTransform( const rpcTransform& tx ) const;

	bool TestPoint( const rpcTransform& tx, const rpcVec3& p ) const;
	bool Raycast( const rpcTransform& tx, rpcRaycastData* raycast ) const;
	void ComputeAABB( const rpcTransform& tx, rpcAABB* aabb ) const;
	void ComputeMass( rpcMassData* md ) const;
	void Render( const rpcTransform& tx, bool awake, rpcRender* render ) const;
};

//--------------------------------------------------------------------------------------------------
// rpcBoxDef
//--------------------------------------------------------------------------------------------------
class rpcBoxDef
{
public:
	rpcBoxDef( )
	{
		// Common default values
		m_friction = r32( 0.4 );
		m_restitution = r32( 0.2 );
		m_density = r32( 1.0 );
		m_sensor = false;
	}

	void Set( const rpcTransform& tx, const rpcVec3& extents );

	void SetFriction( r32 friction );
	void SetRestitution( r32 restitution );
	void SetDensity( r32 density );
	void SetSensor( bool sensor );

private:
	rpcTransform m_tx;
	rpcVec3 m_e;

	r32 m_friction;
	r32 m_restitution;
	r32 m_density;
	bool m_sensor;

	friend class rpcBody;
};

#include "rpcBox.inl"


#endif // RPCBOX_H
