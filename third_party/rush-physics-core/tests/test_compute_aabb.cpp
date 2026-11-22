//--------------------------------------------------------------------------------------------------
// L1 unit test: rpcBox::ComputeAABB (OBB center +/- |R|*e) must match the legacy
// 8-corner min/max reference within 1e-5 on random poses (rpc-refactor.md T3).
// Links only rush-physics-core — no engine, no game.
//--------------------------------------------------------------------------------------------------

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../collision/rpcBox.h"
#include "../math/rpcMath.h"
#include "../math/rpcQuaternion.h"
#include "../math/rpcMat3.h"

namespace {

// Legacy reference: transform all 8 corners and take componentwise min/max.
void ComputeAABBReference( const rpcBox& box, const rpcTransform& tx, rpcAABB* aabb )
{
	rpcTransform world = rpcMul( tx, box.local );

	rpcVec3 v[ 8 ] = {
		rpcVec3( -box.e.x, -box.e.y, -box.e.z ),
		rpcVec3( -box.e.x, -box.e.y,  box.e.z ),
		rpcVec3( -box.e.x,  box.e.y, -box.e.z ),
		rpcVec3( -box.e.x,  box.e.y,  box.e.z ),
		rpcVec3(  box.e.x, -box.e.y, -box.e.z ),
		rpcVec3(  box.e.x, -box.e.y,  box.e.z ),
		rpcVec3(  box.e.x,  box.e.y, -box.e.z ),
		rpcVec3(  box.e.x,  box.e.y,  box.e.z )
	};

	for ( i32 i = 0; i < 8; ++i )
		v[ i ] = rpcMul( world, v[ i ] );

	aabb->min = rpcVec3( RPC_R32_MAX, RPC_R32_MAX, RPC_R32_MAX );
	aabb->max = rpcVec3( -RPC_R32_MAX, -RPC_R32_MAX, -RPC_R32_MAX );

	for ( i32 i = 0; i < 8; ++i )
	{
		aabb->min = rpcMin( aabb->min, v[ i ] );
		aabb->max = rpcMax( aabb->max, v[ i ] );
	}
}

r32 RandRange( r32 lo, r32 hi )
{
	return lo + ( hi - lo ) * ( r32( std::rand( ) ) / r32( RAND_MAX ) );
}

void SetRandomRotation( rpcTransform* tx )
{
	const rpcVec3 axis = rpcNormalize( rpcVec3( RandRange( -1.f, 1.f ), RandRange( -1.f, 1.f ), RandRange( -1.f, 1.f ) ) );
	const rpcQuaternion q( axis, RandRange( -3.14159265f, 3.14159265f ) );
	tx->rotation = q.ToMat3( );
}

}  // namespace

int main( )
{
	std::srand( 20240611 );

	const i32 kTrials = 10000;
	r32 maxError = 0;

	for ( i32 i = 0; i < kTrials; ++i )
	{
		rpcBox box;
		box.e = rpcVec3( RandRange( 0.05f, 3.f ), RandRange( 0.05f, 3.f ), RandRange( 0.05f, 3.f ) );

		rpcTransform bodyTx;
		bodyTx.position = rpcVec3( RandRange( -50.f, 50.f ), RandRange( -50.f, 50.f ), RandRange( -50.f, 50.f ) );
		SetRandomRotation( &bodyTx );

		box.local.position = rpcVec3( RandRange( -2.f, 2.f ), RandRange( -2.f, 2.f ), RandRange( -2.f, 2.f ) );
		SetRandomRotation( &box.local );

		rpcAABB fast, ref;
		box.ComputeAABB( bodyTx, &fast );
		ComputeAABBReference( box, bodyTx, &ref );

		if ( i == 0 )
			std::printf( "sample: fast=[%g %g %g]-[%g %g %g]\n",
				double( fast.min.x ), double( fast.min.y ), double( fast.min.z ),
				double( fast.max.x ), double( fast.max.y ), double( fast.max.z ) );

		for ( i32 c = 0; c < 3; ++c )
		{
			maxError = rpcMax( maxError, rpcAbs( fast.min[ c ] - ref.min[ c ] ) );
			maxError = rpcMax( maxError, rpcAbs( fast.max[ c ] - ref.max[ c ] ) );
		}
	}

	std::printf( "test_compute_aabb: %d trials, max error = %g\n", kTrials, double( maxError ) );

	if ( maxError >= 1e-5f )
	{
		std::printf( "FAIL: max error >= 1e-5\n" );
		return 1;
	}

	std::printf( "PASS\n" );
	return 0;
}
