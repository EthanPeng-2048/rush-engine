//--------------------------------------------------------------------------------------------------
// rpcMat3.inl
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

//--------------------------------------------------------------------------------------------------
// rpcMat3
//--------------------------------------------------------------------------------------------------
inline void rpcIdentity( rpcMat3& m )
{
	m.Set(
		r32( 1.0 ), r32( 0.0 ), r32( 0.0 ),
		r32( 0.0 ), r32( 1.0 ), r32( 0.0 ),
		r32( 0.0 ), r32( 0.0 ), r32( 1.0 )
		);
}

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcRotate( const rpcVec3& x, const rpcVec3& y, const rpcVec3& z )
{
	return rpcMat3( x, y, z );
}

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcTranspose( const rpcMat3& m )
{
	return rpcMat3(
		m.ex.x, m.ey.x, m.ez.x,
		m.ex.y, m.ey.y, m.ez.y,
		m.ex.z, m.ey.z, m.ez.z
		);
}

//--------------------------------------------------------------------------------------------------
inline void rpcZero( rpcMat3& m )
{
	memset( &m, 0, sizeof( r32 ) * 9 );
}

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcDiagonal( r32 a )
{
	return rpcMat3(
		r32( a   ), r32( 0.0 ), r32( 0.0 ),
		r32( 0.0 ), r32( a   ), r32( 0.0 ),
		r32( 0.0 ), r32( 0.0 ), r32( a   )
		);
}

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcDiagonal( r32 a, r32 b, r32 c )
{
	return rpcMat3(
		r32( a   ), r32( 0.0 ), r32( 0.0 ),
		r32( 0.0 ), r32( b   ), r32( 0.0 ),
		r32( 0.0 ), r32( 0.0 ), r32( c   )
		);
}

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcOuterProduct( const rpcVec3& u, const rpcVec3& v )
{
	rpcVec3 a = v * u.x;
	rpcVec3 b = v * u.y;
	rpcVec3 c = v * u.z;

	return rpcMat3(
		a.x, a.y, a.z,
		b.x, b.y, b.z,
		c.x, c.y, c.z
		);
}

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcCovariance( rpcVec3 *points, u32 numPoints )
{
	r32 invNumPoints = r32( 1.0 ) / r32( numPoints );
	rpcVec3 c = rpcVec3( r32( 0.0 ), r32( 0.0 ), r32( 0.0 ) );

	for ( u32 i = 0; i < numPoints; ++i )
		c += points[ i ];

	c /= r32( numPoints );

	r32 m00, m11, m22, m01, m02, m12;
	m00 = m11 = m22 = m01 = m02 = m12 = r32( 0.0 );

	for ( u32 i = 0; i < numPoints; ++i )
	{
		rpcVec3 p = points[ i ] - c;

		m00 += p.x * p.x;
		m11 += p.y * p.y;
		m22 += p.z * p.z;
		m01 += p.x * p.y;
		m02 += p.x * p.z;
		m12 += p.y * p.z;
	}

	r32 m01inv = m01 * invNumPoints;
	r32 m02inv = m02 * invNumPoints;
	r32 m12inv = m12 * invNumPoints;

	return rpcMat3(
		m00 * invNumPoints, m01inv, m02inv,
		m01inv, m11 * invNumPoints, m12inv,
		m02inv, m12inv, m22 * invNumPoints
		);
};

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcInverse( const rpcMat3& m )
{
	rpcVec3 tmp0, tmp1, tmp2;
	r32 detinv;

	tmp0 = rpcCross( m.ey, m.ez );
	tmp1 = rpcCross( m.ez, m.ex );
	tmp2 = rpcCross( m.ex, m.ey );

	detinv = r32( 1.0 ) / rpcDot( m.ez, tmp2 );

	return rpcMat3(
		tmp0.x * detinv, tmp1.x * detinv, tmp2.x * detinv,
		tmp0.y * detinv, tmp1.y * detinv, tmp2.y * detinv,
		tmp0.z * detinv, tmp1.z * detinv, tmp2.z * detinv
		);
}
