//--------------------------------------------------------------------------------------------------
// rpcVec3.inl
//
//	Copyright (c) 2014 Randy Gaul http://www.randygaul.net
//
//	This software is provided 'as-is', without any express or implied
//	warranty. In no event will the authors be held liable for any damages
//	arising from the use of this software.
//
//	Permission is granted to anyone to use this software for any purpose,
//	including commercial applications, and to alter it and redistribute it
//	freely, subject to the following restrictions:
//	  1. The origin of this software must not be misrepresented; you must not
//	     claim that you wrote the original software. If you use this software
//	     in a product, an acknowledgment in the product documentation would be
//	     appreciated but is not required.
//	  2. Altered source versions must be plainly marked as such, and must not
//	     be misrepresented as being the original software.
//	  3. This notice may not be removed or altered from any source distribution.
//--------------------------------------------------------------------------------------------------

#include <cmath>

//--------------------------------------------------------------------------------------------------
// rpcVec3
//--------------------------------------------------------------------------------------------------
inline void rpcIdentity( rpcVec3& v )
{
	v.Set( r32( 0.0 ), r32( 0.0 ), r32( 0.0 ) );
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMul( const rpcVec3& a, const rpcVec3& b )
{
	return rpcVec3( a.x * b.x, a.y * b.y, a.z * b.z );
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcDot( const rpcVec3& a, const rpcVec3& b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcCross( const rpcVec3& a, const rpcVec3& b )
{
	return rpcVec3(
		(a.y * b.z) - (b.y * a.z),
		(b.x * a.z) - (a.x * b.z),
		(a.x * b.y) - (b.x * a.y)
		);
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcLength( const rpcVec3& v )
{
	return std::sqrt( v.x * v.x + v.y * v.y + v.z * v.z );
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcLengthSq( const rpcVec3& v )
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcNormalize( const rpcVec3& v )
{
	r32 l = rpcLength( v );

	if ( l != r32( 0.0 ) )
	{
		r32 inv = r32( 1.0 ) / l;
		return v * inv;
	}

	return v;
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcDistance( const rpcVec3& a, const rpcVec3& b )
{
	r32 xp = a.x - b.x;
	r32 yp = a.y - b.y;
	r32 zp = a.z - b.z;

	return std::sqrt( xp * xp + yp * yp + zp * zp );
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcDistanceSq( const rpcVec3& a, const rpcVec3& b )
{
	r32 xp = a.x - b.x;
	r32 yp = a.y - b.y;
	r32 zp = a.z - b.z;

	return xp * xp + yp * yp + zp * zp;
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcAbs( const rpcVec3& v )
{
	return rpcVec3( rpcAbs( v.x ), rpcAbs( v.y ), rpcAbs( v.z ) );
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMin( const rpcVec3& a, const rpcVec3& b )
{
	return rpcVec3( rpcMin( a.x, b.x ), rpcMin( a.y, b.y ), rpcMin( a.z, b.z ) );
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMax( const rpcVec3& a, const rpcVec3& b )
{
	return rpcVec3( rpcMax( a.x, b.x ), rpcMax( a.y, b.y ), rpcMax( a.z, b.z ) );
}

//--------------------------------------------------------------------------------------------------
inline const r32 rpcMinPerElem( const rpcVec3& a )
{
	return rpcMin( a.x, rpcMin( a.y, a.z ) );
}

//--------------------------------------------------------------------------------------------------
inline const r32 rpcMaxPerElem( const rpcVec3& a )
{
	return rpcMax( a.x, rpcMax( a.y, a.z ) );
}
