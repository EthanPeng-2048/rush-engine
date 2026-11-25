//--------------------------------------------------------------------------------------------------
/**
@file	rpcBox.cpp

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

#include "rpcBox.h"
#include "../math/rpcVec3.h"

//--------------------------------------------------------------------------------------------------
// rpcBox
//--------------------------------------------------------------------------------------------------
rpcTransform rpcBox::WorldTransform( const rpcTransform& tx ) const
{
	if ( !HasLocalRotation( ) )
	{
		rpcTransform world;
		world.rotation = tx.rotation;
		world.position = tx.position + rpcMul( tx.rotation, local.position );
		return world;
	}

	return rpcMul( tx, local );
}

//--------------------------------------------------------------------------------------------------
bool rpcBox::TestPoint( const rpcTransform& tx, const rpcVec3& p ) const
{
	rpcTransform world = WorldTransform( tx );
	rpcVec3 p0 = rpcMulT( world, p );

	for ( int i = 0; i < 3; ++i )
	{
		r32 d = p0[ i ];
		r32 ei = e[ i ];

		if ( d > ei || d < -ei )
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------
bool rpcBox::Raycast( const rpcTransform& tx, rpcRaycastData* raycast ) const
{
	rpcTransform world = WorldTransform( tx );
	rpcVec3 d = rpcMulT( world.rotation, raycast->dir );
	rpcVec3 p = rpcMulT( world, raycast->start );
	const r32 epsilon = r32( 1.0e-8 );
	r32 tmin = 0;
	r32 tmax = raycast->t;

	// t = (e[ i ] - p.[ i ]) / d[ i ]
	r32 t0;
	r32 t1;
	rpcVec3 n0;

	for ( int i = 0; i < 3; ++i )
	{
		// Check for ray parallel to and outside of AABB
		if ( rpcAbs( d[ i ] ) < epsilon )
		{
			// Detect separating axes
			if ( p[ i ] < -e[ i ] || p[ i ] > e[ i ] )
			{
				return false;
			}
		}

		else
		{
			r32 d0 = r32( 1.0 ) / d[ i ];
			r32 s = rpcSign( d[ i ] );
			r32 ei = e[ i ] * s;
			rpcVec3 n( 0, 0, 0 );
			n[ i ] = -s;

			t0 = -(ei + p[ i ]) * d0;
			t1 = (ei - p[ i ]) * d0;

			if ( t0 > tmin )
			{
				n0 = n;
				tmin = t0;
			}

			tmax = rpcMin( tmax, t1 );

			if ( tmin > tmax )
			{
				return false;
			}
		}
	}

	raycast->normal = rpcMul( world.rotation, n0 ); 
	raycast->toi = tmin;

	return true;
}

//--------------------------------------------------------------------------------------------------
void rpcBox::ComputeAABB( const rpcTransform& tx, rpcAABB* aabb ) const
{
	rpcTransform world = WorldTransform( tx );

	// OBB AABB: center +/- |R| * e (absolute rotation columns times extents),
	// equivalent to the 8-corner min/max at a fraction of the cost.
	const rpcMat3& R = world.rotation;
	rpcVec3 extent(
		rpcAbs( R.ex.x ) * e.x + rpcAbs( R.ey.x ) * e.y + rpcAbs( R.ez.x ) * e.z,
		rpcAbs( R.ex.y ) * e.x + rpcAbs( R.ey.y ) * e.y + rpcAbs( R.ez.y ) * e.z,
		rpcAbs( R.ex.z ) * e.x + rpcAbs( R.ey.z ) * e.y + rpcAbs( R.ez.z ) * e.z
	);

	aabb->min = world.position - extent;
	aabb->max = world.position + extent;
}

//--------------------------------------------------------------------------------------------------
void rpcBox::ComputeMass( rpcMassData* md ) const
{
	// Calculate inertia tensor
	r32 ex2 = r32( 4.0 ) * e.x * e.x;
	r32 ey2 = r32( 4.0 ) * e.y * e.y;
	r32 ez2 = r32( 4.0 ) * e.z * e.z;
	r32 mass = r32( 8.0 ) * e.x * e.y * e.z * density;
	r32 x = r32( 1.0 / 12.0 ) * mass * (ey2 + ez2);
	r32 y = r32( 1.0 / 12.0 ) * mass * (ex2 + ez2);
	r32 z = r32( 1.0 / 12.0 ) * mass * (ex2 + ey2);
	rpcMat3 I = rpcDiagonal( x, y, z );

	// Transform tensor to local space
	I = local.rotation * I * rpcTranspose( local.rotation );
	rpcMat3 identity;
	rpcIdentity( identity );
	I += (identity * rpcDot( local.position, local.position ) - rpcOuterProduct( local.position, local.position )) * mass;

	md->center = local.position;
	md->inertia = I;
	md->mass = mass;
}

//--------------------------------------------------------------------------------------------------
const i32 kBoxIndices[ 36 ] = {
	1 - 1, 7 - 1, 5 - 1,
	1 - 1, 3 - 1, 7 - 1,
	1 - 1, 4 - 1, 3 - 1,
	1 - 1, 2 - 1, 4 - 1,
	3 - 1, 8 - 1, 7 - 1,
	3 - 1, 4 - 1, 8 - 1,
	5 - 1, 7 - 1, 8 - 1,
	5 - 1, 8 - 1, 6 - 1,
	1 - 1, 5 - 1, 6 - 1,
	1 - 1, 6 - 1, 2 - 1,
	2 - 1, 6 - 1, 8 - 1,
	2 - 1, 8 - 1, 4 - 1
};

//--------------------------------------------------------------------------------------------------
void rpcBox::Render( const rpcTransform& tx, bool awake, rpcRender* render ) const
{
	rpcTransform world = rpcMul( tx, local );

	rpcVec3 vertices[ 8 ] = {
		rpcVec3( -e.x, -e.y, -e.z ),
		rpcVec3( -e.x, -e.y,  e.z ),
		rpcVec3( -e.x,  e.y, -e.z ),
		rpcVec3( -e.x,  e.y,  e.z ),
		rpcVec3(  e.x, -e.y, -e.z ),
		rpcVec3(  e.x, -e.y,  e.z ),
		rpcVec3(  e.x,  e.y, -e.z ),
		rpcVec3(  e.x,  e.y,  e.z )
	};

	for ( i32 i = 0; i < 36; i += 3 )
	{
		rpcVec3 a = rpcMul( world, vertices[ kBoxIndices[ i ] ] );
		rpcVec3 b = rpcMul( world, vertices[ kBoxIndices[ i + 1 ] ] );
		rpcVec3 c = rpcMul( world, vertices[ kBoxIndices[ i + 2 ] ] );

		rpcVec3 n = rpcNormalize( rpcCross( b - a, c - a ) );

		//render->SetPenColor( 0.2f, 0.4f, 0.7f, 0.5f );
		//render->SetPenPosition( a.x, a.y, a.z );
		//render->Line( b.x, b.y, b.z );
		//render->Line( c.x, c.y, c.z );
		//render->Line( a.x, a.y, a.z );

		render->SetTriNormal( n.x, n.y, n.z );
		render->Triangle( a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z );
	}
}
