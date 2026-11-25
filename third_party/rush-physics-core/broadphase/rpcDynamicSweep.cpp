//--------------------------------------------------------------------------------------------------
/**
@file	rpcDynamicSweep.cpp
*/
//--------------------------------------------------------------------------------------------------

#include "rpcDynamicSweep.h"
#include "../scene/rpcScene.h"
#include "../dynamics/rpcBody.h"
#include "../collision/rpcBox.h"

namespace {

// Conservative segment/AABB test (same one the broadphase tree uses).
bool SegmentMeetsAabb( const rpcAABB& aabb, const rpcVec3& p0, const rpcVec3& p1 )
{
	const r32 epsilon = r32( 1.0e-6 );
	const rpcVec3 e = aabb.max - aabb.min;
	const rpcVec3 d = p1 - p0;
	const rpcVec3 m = p0 + p1 - aabb.min - aabb.max;

	const r32 adx = rpcAbs( d.x ) + epsilon;
	if ( rpcAbs( m.x ) > e.x + adx )
		return false;

	const r32 ady = rpcAbs( d.y ) + epsilon;
	if ( rpcAbs( m.y ) > e.y + ady )
		return false;

	const r32 adz = rpcAbs( d.z ) + epsilon;
	if ( rpcAbs( m.z ) > e.z + adz )
		return false;

	if ( rpcAbs( m.y * d.z - m.z * d.y ) > e.y * adz + e.z * ady )
		return false;

	if ( rpcAbs( m.z * d.x - m.x * d.z ) > e.x * adz + e.z * adx )
		return false;

	if ( rpcAbs( m.x * d.y - m.y * d.x ) > e.x * ady + e.y * adx )
		return false;

	return true;
}

constexpr i32 kMaxBatchRays = 32;

}  // namespace

//--------------------------------------------------------------------------------------------------
void rpcDynamicSweep::Raycast( rpcQueryCallback* cb, rpcRaycastData& ray, const rpcBody* ignore ) const
{
	if ( !m_source )
		return;

	m_lastCandidates = 0;

	const rpcVec3 p0 = ray.start;
	const rpcVec3 p1 = p0 + ray.dir * ray.t;

	rpcBody** bodies = m_source->Bodies( );
	const i32 count = m_source->BodyCount( );
	[[assume( count >= 0 )]];

	// Hot loop over proxy AABBs: contiguous, branch light, no dereference until
	// a body actually meets the segment.
	for ( i32 i = 0; i < count; ++i )
	{
		rpcBody* body = bodies[ i ];
		if ( !body || body == ignore )
			continue;

		// Most bodies are far away: rejection is the common case.
		if ( !SegmentMeetsAabb( body->GetFatAABB( ), p0, p1 ) ) [[likely]]
			continue;

		++m_lastCandidates;

		const rpcTransform tx = body->GetTransform( );
		for ( rpcBox* box = body->GetBoxes( ); box; box = box->next )
		{
			if ( !SegmentMeetsAabb( box->fatAABB, p0, p1 ) )
				continue;

			if ( box->Raycast( tx, &ray ) )
			{
				if ( !cb->ReportShape( box ) )
					return;
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------
