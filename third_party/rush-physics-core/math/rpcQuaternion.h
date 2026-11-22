//--------------------------------------------------------------------------------------------------
/**
@file	rpcQuaternion.h

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

#ifndef RPCQUATERNION_H
#define RPCQUATERNION_H

#include "../common/rpcTypes.h"
#include "rpcVec3.h"

//--------------------------------------------------------------------------------------------------
// rpcQuaternion
//--------------------------------------------------------------------------------------------------
struct rpcMat3;

class rpcQuaternion
{
public:
	union
	{
		r32 v[ 4 ];

		struct
		{
			r32 x;
			r32 y;
			r32 z;

			r32 w;
		};
	};

	rpcQuaternion( );
	rpcQuaternion( r32 a, r32 b, r32 c, r32 d );
	rpcQuaternion( const rpcVec3& axis, r32 radians );

	void Set( const rpcVec3& axis, r32 radians );
	void ToAxisAngle( rpcVec3* axis, r32* angle ) const;
	void Integrate( const rpcVec3& dv, r32 dt );

	const rpcQuaternion operator*( const rpcQuaternion& rhs ) const;
	rpcQuaternion& operator*=( const rpcQuaternion& rhs );

	const rpcMat3 ToMat3( void ) const;
};

//--------------------------------------------------------------------------------------------------
inline const rpcQuaternion rpcNormalize( const rpcQuaternion& q )
{
	r32 x = q.x;
	r32 y = q.y;
	r32 z = q.z;
	r32 w = q.w;

	r32 d = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

	if( d == 0 )
		w = r32( 1.0 );

	d = r32( 1.0 ) / std::sqrt( d );

	if ( d > r32( 1.0e-8 ) )
	{
		x *= d;
		y *= d;
		z *= d;
		w *= d;
	}

	return rpcQuaternion( x, y, z, w );
}

#endif // RPCQUATERNION_H
