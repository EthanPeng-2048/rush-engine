//--------------------------------------------------------------------------------------------------
/**
@file	rpcCollide.cpp

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

#include "rpcCollide.h"
#include "../dynamics/rpcBody.h"
#include "../dynamics/rpcContact.h"

//--------------------------------------------------------------------------------------------------
// qBoxtoBox
//--------------------------------------------------------------------------------------------------
inline bool rpcTrackFaceAxis( i32* axis, i32 n, r32 s, r32* sMax, const rpcVec3& normal, rpcVec3* axisNormal )
{
	if ( s > r32( 0.0 ) )
		return true;

	if ( s > *sMax )
	{
		*sMax = s;
		*axis = n;
		*axisNormal = normal;
	}

	return false;
}

//--------------------------------------------------------------------------------------------------
inline bool rpcTrackEdgeAxis( i32* axis, i32 n, r32 s, r32* sMax, const rpcVec3& normal, rpcVec3* axisNormal )
{
	if ( s > r32( 0.0 ) )
		return true;

	r32 l = r32( 1.0 ) / rpcLength( normal );
	s *= l;

	if ( s > *sMax )
	{
		*sMax = s;
		*axis = n;
		*axisNormal = normal * l;
	}

	return false;
}

//--------------------------------------------------------------------------------------------------
struct rpcClipVertex
{
	rpcClipVertex( )
	{
		f.key = ~0;
	}

	rpcVec3 v;
	rpcFeaturePair f;
};

//--------------------------------------------------------------------------------------------------
void rpcComputeReferenceEdgesAndBasis( const rpcVec3& eR, const rpcTransform& rtx, rpcVec3 n, i32 axis, u8* out, rpcMat3* basis, rpcVec3* e )
{
	n = rpcMulT( rtx.rotation, n );

	if ( axis >= 3 )
		axis -= 3;

	switch ( axis )
	{
	case 0:
		if ( n.x > r32( 0.0 ) )
		{
			out[ 0 ] = 1;
			out[ 1 ] = 8;
			out[ 2 ] = 7;
			out[ 3 ] = 9;

			e->Set( eR.y, eR.z, eR.x );
			basis->SetRows( rtx.rotation.ey, rtx.rotation.ez, rtx.rotation.ex );
		}

		else
		{
			out[ 0 ] = 11;
			out[ 1 ] = 3;
			out[ 2 ] = 10;
			out[ 3 ] = 5;

			e->Set( eR.z, eR.y, eR.x );
			basis->SetRows( rtx.rotation.ez, rtx.rotation.ey, -rtx.rotation.ex );
		}
		break;

	case 1:
		if ( n.y > r32( 0.0 ) )
		{
			out[ 0 ] = 0;
			out[ 1 ] = 1;
			out[ 2 ] = 2;
			out[ 3 ] = 3;

			e->Set( eR.z, eR.x, eR.y );
			basis->SetRows( rtx.rotation.ez, rtx.rotation.ex, rtx.rotation.ey );
		}

		else
		{
			out[ 0 ] = 4;
			out[ 1 ] = 5;
			out[ 2 ] = 6;
			out[ 3 ] = 7;

			e->Set( eR.z, eR.x, eR.y );
			basis->SetRows( rtx.rotation.ez, -rtx.rotation.ex, -rtx.rotation.ey );
		}
		break;

	case 2:
		if ( n.z > r32( 0.0 ) )
		{
			out[ 0 ] = 11;
			out[ 1 ] = 4;
			out[ 2 ] = 8;
			out[ 3 ] = 0;

			e->Set( eR.y, eR.x, eR.z );
			basis->SetRows( -rtx.rotation.ey, rtx.rotation.ex, rtx.rotation.ez );
		}

		else
		{
			out[ 0 ] = 6;
			out[ 1 ] = 10;
			out[ 2 ] = 2;
			out[ 3 ] = 9;

			e->Set( eR.y, eR.x, eR.z );
			basis->SetRows( -rtx.rotation.ey, -rtx.rotation.ex, -rtx.rotation.ez );
		}
		break;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcComputeIncidentFace( const rpcTransform& itx, const rpcVec3& e, rpcVec3 n, rpcClipVertex* out )
{
	n = -rpcMulT( itx.rotation, n );
	rpcVec3 absN = rpcAbs( n );

	if ( absN.x > absN.y && absN.x > absN.z )
	{
		if ( n.x > r32( 0.0 ) )
		{
			out[ 0 ].v.Set(  e.x,  e.y, -e.z );
			out[ 1 ].v.Set(  e.x,  e.y,  e.z );
			out[ 2 ].v.Set(  e.x, -e.y,  e.z );
			out[ 3 ].v.Set(  e.x, -e.y, -e.z );

			out[ 0 ].f.inI = 9;
			out[ 0 ].f.outI = 1;
			out[ 1 ].f.inI = 1;
			out[ 1 ].f.outI = 8;
			out[ 2 ].f.inI = 8;
			out[ 2 ].f.outI = 7;
			out[ 3 ].f.inI = 7;
			out[ 3 ].f.outI = 9;
		}

		else
		{
			out[ 0 ].v.Set( -e.x, -e.y,  e.z );
			out[ 1 ].v.Set( -e.x,  e.y,  e.z );
			out[ 2 ].v.Set( -e.x,  e.y, -e.z );
			out[ 3 ].v.Set( -e.x, -e.y, -e.z );

			out[ 0 ].f.inI = 5;
			out[ 0 ].f.outI = 11;
			out[ 1 ].f.inI = 11;
			out[ 1 ].f.outI = 3;
			out[ 2 ].f.inI = 3;
			out[ 2 ].f.outI = 10;
			out[ 3 ].f.inI = 10;
			out[ 3 ].f.outI = 5;
		}
	}

	else if ( absN.y > absN.x && absN.y > absN.z )
	{
		if ( n.y > r32( 0.0 ) )
		{
			out[ 0 ].v.Set( -e.x,  e.y,  e.z );
			out[ 1 ].v.Set(  e.x,  e.y,  e.z );
			out[ 2 ].v.Set(  e.x,  e.y, -e.z );
			out[ 3 ].v.Set( -e.x,  e.y, -e.z );

			out[ 0 ].f.inI = 3;
			out[ 0 ].f.outI = 0;
			out[ 1 ].f.inI = 0;
			out[ 1 ].f.outI = 1;
			out[ 2 ].f.inI = 1;
			out[ 2 ].f.outI = 2;
			out[ 3 ].f.inI = 2;
			out[ 3 ].f.outI = 3;
		}

		else
		{
			out[ 0 ].v.Set(  e.x, -e.y,  e.z );
			out[ 1 ].v.Set( -e.x, -e.y,  e.z );
			out[ 2 ].v.Set( -e.x, -e.y, -e.z );
			out[ 3 ].v.Set(  e.x, -e.y, -e.z );

			out[ 0 ].f.inI = 7;
			out[ 0 ].f.outI = 4;
			out[ 1 ].f.inI = 4;
			out[ 1 ].f.outI = 5;
			out[ 2 ].f.inI = 5;
			out[ 2 ].f.outI = 6;
			out[ 3 ].f.inI = 5;
			out[ 3 ].f.outI = 6;
		}
	}

	else
	{
		if ( n.z > r32( 0.0 ) )
		{
			out[ 0 ].v.Set( -e.x,  e.y,  e.z );
			out[ 1 ].v.Set( -e.x, -e.y,  e.z );
			out[ 2 ].v.Set(  e.x, -e.y,  e.z );
			out[ 3 ].v.Set(  e.x,  e.y,  e.z );

			out[ 0 ].f.inI = 0;
			out[ 0 ].f.outI = 11;
			out[ 1 ].f.inI = 11;
			out[ 1 ].f.outI = 4;
			out[ 2 ].f.inI = 4;
			out[ 2 ].f.outI = 8;
			out[ 3 ].f.inI = 8;
			out[ 3 ].f.outI = 0;
		}

		else
		{
			out[ 0 ].v.Set(  e.x, -e.y, -e.z );
			out[ 1 ].v.Set( -e.x, -e.y, -e.z );
			out[ 2 ].v.Set( -e.x,  e.y, -e.z );
			out[ 3 ].v.Set(  e.x,  e.y, -e.z );

			out[ 0 ].f.inI = 9;
			out[ 0 ].f.outI = 6;
			out[ 1 ].f.inI = 6;
			out[ 1 ].f.outI = 10;
			out[ 2 ].f.inI = 10;
			out[ 2 ].f.outI = 2;
			out[ 3 ].f.inI = 2;
			out[ 3 ].f.outI = 9;
		}
	}

	for ( i32 i = 0; i < 4; ++i )
		out[ i ].v = rpcMul( itx, out[ i ].v );
}

//--------------------------------------------------------------------------------------------------
#define InFront( a ) \
	((a) < r32( 0.0 ))

#define Behind( a ) \
	((a) >= r32( 0.0 ))

#define On( a ) \
	((a) < r32( 0.005 ) && (a) > -r32( 0.005 ))

i32 rpcOrthographic( r32 sign, r32 e, i32 axis, i32 clipEdge, rpcClipVertex* in, i32 inCount, rpcClipVertex* out )
{
	i32 outCount = 0;
	rpcClipVertex a = in[ inCount - 1 ];

	for ( i32 i = 0; i < inCount; ++i )
	{
		rpcClipVertex b = in[ i ];

		r32 da = sign * a.v[ axis ] - e;
		r32 db = sign * b.v[ axis ] - e;

		rpcClipVertex cv;

		// B
		if ( ((InFront( da ) && InFront( db )) || On( da ) || On( db )) )
		{
			assert( outCount < 8 );
			out[ outCount++ ] = b;
		}

		// I
		else if ( InFront( da ) && Behind( db ) )
		{
			cv.f = b.f;
			cv.v = a.v + (b.v - a.v) * (da / (da - db));
			cv.f.outR = clipEdge;
			cv.f.outI = 0;
			assert( outCount < 8 );
			out[ outCount++ ] = cv;
		}

		// I, B
		else if ( Behind( da ) && InFront( db ) )
		{
			cv.f = a.f;
			cv.v = a.v + (b.v - a.v) * (da / (da - db));
			cv.f.inR = clipEdge;
			cv.f.inI = 0;
			assert( outCount < 8 );
			out[ outCount++ ] = cv;

			assert( outCount < 8 );
			out[ outCount++ ] = b;
		}

		a = b;
	}

	return outCount;
}

//--------------------------------------------------------------------------------------------------
// Resources (also see rpcBoxtoBox's resources):
// http://www.randygaul.net/2013/10/27/sutherland-hodgman-clipping/
i32 rpcClip( const rpcVec3& rPos, const rpcVec3& e, u8* clipEdges, const rpcMat3& basis, rpcClipVertex* incident, rpcClipVertex* outVerts, r32* outDepths )
{
	i32 inCount = 4;
	i32 outCount;
	rpcClipVertex in[ 8 ];
	rpcClipVertex out[ 8 ];

	for ( i32 i = 0; i < 4; ++i )
		in[ i ].v = rpcMulT( basis, incident[ i ].v - rPos );

	outCount = rpcOrthographic( r32( 1.0 ), e.x, 0, clipEdges[ 0 ], in, inCount, out );

	if ( !outCount )
		return 0;

	inCount = rpcOrthographic( r32( 1.0 ), e.y, 1, clipEdges[ 1 ], out, outCount, in );

	if ( !inCount )
		return 0;

	outCount = rpcOrthographic( r32( -1.0 ), e.x, 0, clipEdges[ 2 ], in, inCount, out );

	if ( !outCount )
		return 0;

	inCount = rpcOrthographic( r32( -1.0 ), e.y, 1, clipEdges[ 3 ], out, outCount, in );

	// Keep incident vertices behind the reference face
	outCount = 0;
	for ( i32 i = 0; i < inCount; ++i )
	{
		r32 d = in[ i ].v.z - e.z;

		if ( d <= r32( 0.0 ) )
		{
			outVerts[ outCount ].v = rpcMul( basis, in[ i ].v ) + rPos;
			outVerts[ outCount ].f = in[ i ].f;
			outDepths[ outCount++ ] = d;
		}
	}

	assert( outCount <= 8 );

	return outCount;
}

//--------------------------------------------------------------------------------------------------
inline void rpcEdgesContact( rpcVec3 *CA, rpcVec3 *CB, const rpcVec3& PA, const rpcVec3& QA, const rpcVec3& PB, const rpcVec3& QB )
{
	rpcVec3 DA = QA - PA;
	rpcVec3 DB = QB - PB;
	rpcVec3 r = PA - PB;
	r32 a = rpcDot( DA, DA );
	r32 e = rpcDot( DB, DB );
	r32 f = rpcDot( DB, r );
	r32 c = rpcDot( DA, r );

	r32 b = rpcDot( DA, DB );
	r32 denom = a * e - b * b;

	r32 TA = (b * f - c * e) / denom;
	r32 TB = (b * TA + f) / e;

	*CA = PA + DA * TA;
	*CB = PB + DB * TB;
}

//--------------------------------------------------------------------------------------------------
void rpcSupportEdge( const rpcTransform& tx, const rpcVec3& e, rpcVec3 n, rpcVec3* aOut, rpcVec3* bOut )
{
	n = rpcMulT( tx.rotation, n );
	rpcVec3 absN = rpcAbs( n );
	rpcVec3 a, b;

	// x > y
	if ( absN.x > absN.y )
	{
		// x > y > z
		if ( absN.y > absN.z )
		{
			a.Set( e.x, e.y, e.z );
			b.Set( e.x, e.y, -e.z );
		}

		// x > z > y || z > x > y
		else
		{
			a.Set( e.x, e.y, e.z );
			b.Set( e.x, -e.y, e.z );
		}
	}

	// y > x
	else
	{
		// y > x > z
		if ( absN.x > absN.z )
		{
			a.Set( e.x, e.y, e.z );
			b.Set( e.x, e.y, -e.z );
		}

		// z > y > x || y > z > x
		else
		{
			a.Set( e.x, e.y, e.z );
			b.Set( -e.x, e.y, e.z );
		}
	}

	r32 signx = rpcSign( n.x );
	r32 signy = rpcSign( n.y );
	r32 signz = rpcSign( n.z );

	a.x *= signx;
	a.y *= signy;
	a.z *= signz;
	b.x *= signx;
	b.y *= signy;
	b.z *= signz;

	*aOut = rpcMul( tx, a );
	*bOut = rpcMul( tx, b );
}

//--------------------------------------------------------------------------------------------------
// Resources:
// http://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/
// https://box2d.googlecode.com/files/GDC2007_ErinCatto.zip
// https://box2d.googlecode.com/files/Box2D_Lite.zip
void rpcBoxtoBox( rpcManifold* m, rpcBox* a, rpcBox* b )
{
	rpcTransform atx = a->body->GetTransform( );
	rpcTransform btx = b->body->GetTransform( );
	rpcTransform aL = a->local;
	rpcTransform bL = b->local;
	atx = rpcMul( atx, aL );
	btx = rpcMul( btx, bL );
	rpcVec3 eA = a->e;
	rpcVec3 eB = b->e;

	// B's frame in A's space
	rpcMat3 C = rpcTranspose( atx.rotation ) * btx.rotation;

	rpcMat3 absC;
	bool parallel = false;
	const r32 kCosTol = r32( 1.0e-6 );
	for ( i32 i = 0; i < 3; ++i )
	{
		for ( i32 j = 0; j < 3; ++j )
		{
			r32 val = rpcAbs( C[ i ][ j ] );
			absC[ i ][ j ] = val;

			if ( val + kCosTol >= r32( 1.0 ) )
				parallel = true;
		}
	}

	// Vector from center A to center B in A's space
	rpcVec3 t = rpcMulT( atx.rotation, btx.position - atx.position );

	// Query states
	r32 s;
	r32 aMax = -RPC_R32_MAX;
	r32 bMax = -RPC_R32_MAX;
	r32 eMax = -RPC_R32_MAX;
	i32 aAxis = ~0;
	i32 bAxis = ~0;
	i32 eAxis = ~0;
	rpcVec3 nA;
	rpcVec3 nB;
	rpcVec3 nE;

	// Face axis checks

	// a's x axis
	s = rpcAbs( t.x ) - (eA.x + rpcDot( absC.Column0( ), eB ));
	if ( rpcTrackFaceAxis( &aAxis, 0, s, &aMax, atx.rotation.ex, &nA ) )
		return;

	// a's y axis
	s = rpcAbs( t.y ) - (eA.y + rpcDot( absC.Column1( ), eB ));
	if ( rpcTrackFaceAxis( &aAxis, 1, s, &aMax, atx.rotation.ey, &nA ) )
		return;

	// a's z axis
	s = rpcAbs( t.z ) - (eA.z + rpcDot( absC.Column2( ), eB ));
	if ( rpcTrackFaceAxis( &aAxis, 2, s, &aMax, atx.rotation.ez, &nA ) )
		return;

	// b's x axis
	s = rpcAbs( rpcDot( t, C.ex ) ) - (eB.x + rpcDot( absC.ex, eA ));
	if ( rpcTrackFaceAxis( &bAxis, 3, s, &bMax, btx.rotation.ex, &nB ) )
		return;

	// b's y axis
	s = rpcAbs( rpcDot( t, C.ey ) ) - (eB.y + rpcDot( absC.ey, eA ));
	if ( rpcTrackFaceAxis( &bAxis, 4, s, &bMax, btx.rotation.ey, &nB ) )
		return;

	// b's z axis
	s = rpcAbs( rpcDot( t, C.ez ) ) - (eB.z + rpcDot( absC.ez, eA ));
	if ( rpcTrackFaceAxis( &bAxis, 5, s, &bMax, btx.rotation.ez, &nB ) )
		return;

	if ( !parallel )
	{
		// Edge axis checks
		r32 rA;
		r32 rB;

		// Cross( a.x, b.x )
		rA = eA.y * absC[ 0 ][ 2 ] + eA.z * absC[ 0 ][ 1 ];
		rB = eB.y * absC[ 2 ][ 0 ] + eB.z * absC[ 1 ][ 0 ];
		s = rpcAbs( t.z * C[ 0 ][ 1 ] - t.y * C[ 0 ][ 2 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 6, s, &eMax, rpcVec3( r32( 0.0 ), -C[ 0 ][ 2 ], C[ 0 ][ 1 ] ), &nE ) )
			return;

		// Cross( a.x, b.y )
		rA = eA.y * absC[ 1 ][ 2 ] + eA.z * absC[ 1 ][ 1 ];
		rB = eB.x * absC[ 2 ][ 0 ] + eB.z * absC[ 0 ][ 0 ];
		s = rpcAbs( t.z * C[ 1 ][ 1 ] - t.y * C[ 1 ][ 2 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 7, s, &eMax, rpcVec3( r32( 0.0 ), -C[ 1 ][ 2 ], C[ 1 ][ 1 ] ), &nE ) )
			return;

		// Cross( a.x, b.z )
		rA = eA.y * absC[ 2 ][ 2 ] + eA.z * absC[ 2 ][ 1 ];
		rB = eB.x * absC[ 1 ][ 0 ] + eB.y * absC[ 0 ][ 0 ];
		s = rpcAbs( t.z * C[ 2 ][ 1 ] - t.y * C[ 2 ][ 2 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 8, s, &eMax, rpcVec3( r32( 0.0 ), -C[ 2 ][ 2 ], C[ 2 ][ 1 ] ), &nE ) )
			return;

		// Cross( a.y, b.x )
		rA = eA.x * absC[ 0 ][ 2 ] + eA.z * absC[ 0 ][ 0 ];
		rB = eB.y * absC[ 2 ][ 1 ] + eB.z * absC[ 1 ][ 1 ];
		s = rpcAbs( t.x * C[ 0 ][ 2 ] - t.z * C[ 0 ][ 0 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 9, s, &eMax, rpcVec3( C[ 0 ][ 2 ], r32( 0.0 ), -C[ 0 ][ 0 ] ), &nE ) )
			return;

		// Cross( a.y, b.y )
		rA = eA.x * absC[ 1 ][ 2 ] + eA.z * absC[ 1 ][ 0 ];
		rB = eB.x * absC[ 2 ][ 1 ] + eB.z * absC[ 0 ][ 1 ];
		s = rpcAbs( t.x * C[ 1 ][ 2 ] - t.z * C[ 1 ][ 0 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 10, s, &eMax, rpcVec3( C[ 1 ][ 2 ], r32( 0.0 ), -C[ 1 ][ 0 ] ), &nE ) )
			return;

		// Cross( a.y, b.z )
		rA = eA.x * absC[ 2 ][ 2 ] + eA.z * absC[ 2 ][ 0 ];
		rB = eB.x * absC[ 1 ][ 1 ] + eB.y * absC[ 0 ][ 1 ];
		s = rpcAbs( t.x * C[ 2 ][ 2 ] - t.z * C[ 2 ][ 0 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 11, s, &eMax, rpcVec3( C[ 2 ][ 2 ], r32( 0.0 ), -C[ 2 ][ 0 ] ), &nE ) )
			return;

		// Cross( a.z, b.x )
		rA = eA.x * absC[ 0 ][ 1 ] + eA.y * absC[ 0 ][ 0 ];
		rB = eB.y * absC[ 2 ][ 2 ] + eB.z * absC[ 1 ][ 2 ];
		s = rpcAbs( t.y * C[ 0 ][ 0 ] - t.x * C[ 0 ][ 1 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 12, s, &eMax, rpcVec3( -C[ 0 ][ 1 ], C[ 0 ][ 0 ], r32( 0.0 ) ), &nE ) )
			return;

		// Cross( a.z, b.y )
		rA = eA.x * absC[ 1 ][ 1 ] + eA.y * absC[ 1 ][ 0 ];
		rB = eB.x * absC[ 2 ][ 2 ] + eB.z * absC[ 0 ][ 2 ];
		s = rpcAbs( t.y * C[ 1 ][ 0 ] - t.x * C[ 1 ][ 1 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 13, s, &eMax, rpcVec3( -C[ 1 ][ 1 ], C[ 1 ][ 0 ], r32( 0.0 ) ), &nE ) )
			return;

		// Cross( a.z, b.z )
		rA = eA.x * absC[ 2 ][ 1 ] + eA.y * absC[ 2 ][ 0 ];
		rB = eB.x * absC[ 1 ][ 2 ] + eB.y * absC[ 0 ][ 2 ];
		s = rpcAbs( t.y * C[ 2 ][ 0 ] - t.x * C[ 2 ][ 1 ] ) - (rA + rB);
		if ( rpcTrackEdgeAxis( &eAxis, 14, s, &eMax, rpcVec3( -C[ 2 ][ 1 ], C[ 2 ][ 0 ], r32( 0.0 ) ), &nE ) )
			return;
	}

	// Artificial axis bias to improve frame coherence
	const r32 kRelTol = r32( 0.95 );
	const r32 kAbsTol = r32( 0.01 );
	i32 axis;
	r32 sMax;
	rpcVec3 n;
	r32 faceMax = rpcMax( aMax, bMax );
	if ( kRelTol * eMax > faceMax + kAbsTol )
	{
		axis = eAxis;
		sMax = eMax;
		n = nE;
	}

	else
	{
		if ( kRelTol * bMax > aMax + kAbsTol )
		{
			axis = bAxis;
			sMax = bMax;
			n = nB;
		}

		else
		{
			axis = aAxis;
			sMax = aMax;
			n = nA;
		}
	}

	if ( rpcDot( n, btx.position - atx.position ) < r32( 0.0 ) )
		n = -n;

	if ( axis == ~0 )
		return;

	if ( axis < 6 )
	{
		rpcTransform rtx;
		rpcTransform itx;
		rpcVec3 eR;
		rpcVec3 eI;
		bool flip;

		if ( axis < 3 )
		{
			rtx = atx;
			itx = btx;
			eR = eA;
			eI = eB;
			flip = false;
		}

		else
		{
			rtx = btx;
			itx = atx;
			eR = eB;
			eI = eA;
			flip = true;
			n = -n;
		}

		// Compute reference and incident edge information necessary for clipping
		rpcClipVertex incident[ 4 ];
		rpcComputeIncidentFace( itx, eI, n, incident );
		u8 clipEdges[ 4 ];
		rpcMat3 basis;
		rpcVec3 e;
		rpcComputeReferenceEdgesAndBasis( eR, rtx, n, axis, clipEdges, &basis, &e );

		// Clip the incident face against the reference face side planes
		rpcClipVertex out[ 8 ];
		r32 depths[ 8 ];
		i32 outNum;
		outNum = rpcClip( rtx.position, e, clipEdges, basis, incident, out, depths );

		if ( outNum )
		{
			m->contactCount = outNum;
			m->normal = flip ? -n : n;

			for ( i32 i = 0; i < outNum; ++i )
			{
				rpcContact* c = m->contacts + i;

				rpcFeaturePair pair = out[ i ].f;

				if ( flip )
				{
					std::swap( pair.inI, pair.inR );
					std::swap( pair.outI, pair.outR );
				}

				c->fp = out[ i ].f;
				c->position = out[ i ].v;
				c->penetration = depths[ i ];
			}
		}
	}

	else
	{
		n = atx.rotation * n;

		if ( rpcDot( n, btx.position - atx.position ) < r32( 0.0 ) )
			n = -n;

		rpcVec3 PA, QA;
		rpcVec3 PB, QB;
		rpcSupportEdge( atx, eA, n, &PA, &QA );
		rpcSupportEdge( btx, eB, -n, &PB, &QB );

		rpcVec3 CA, CB;
		rpcEdgesContact( &CA, &CB, PA, QA, PB, QB );

		m->normal = n;
		m->contactCount = 1;

		rpcContact* c = m->contacts;
		rpcFeaturePair pair;
		pair.key = axis;
		c->fp = pair;
		c->penetration = sMax;
		c->position = (CA + CB) * r32( 0.5 );
	}
}
