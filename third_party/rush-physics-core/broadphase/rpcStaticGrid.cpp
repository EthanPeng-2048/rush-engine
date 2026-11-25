//--------------------------------------------------------------------------------------------------
/**
@file	rpcStaticGrid.cpp
*/
//--------------------------------------------------------------------------------------------------

#include <algorithm>
#include <cmath>

#include "rpcStaticGrid.h"
#include "../scene/rpcScene.h"
#include "../collision/rpcBox.h"
#include "../dynamics/rpcBody.h"

namespace {

// Same conservative segment/AABB test the broadphase tree uses for pruning, so
// both strategies reject exactly the same shapes (see rpcRayIndex.h contract).
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

// Clip a segment against the XZ rectangle [lo, hi] (Liang-Barsky). Returns
// false when the segment misses the rectangle entirely.
bool ClipSegmentXz( const rpcVec3& p0, const rpcVec3& p1, r32 loX, r32 loZ, r32 hiX, r32 hiZ,
                    r32* tEnter, r32* tExit )
{
	r32 t0 = r32( 0.0 );
	r32 t1 = r32( 1.0 );
	const r32 dirs[ 2 ] = { p1.x - p0.x, p1.z - p0.z };
	const r32 origins[ 2 ] = { p0.x, p0.z };
	const r32 lo[ 2 ] = { loX, loZ };
	const r32 hi[ 2 ] = { hiX, hiZ };

	for ( i32 axis = 0; axis < 2; ++axis )
	{
		if ( rpcAbs( dirs[ axis ] ) < r32( 1.0e-9 ) )
		{
			if ( origins[ axis ] < lo[ axis ] || origins[ axis ] > hi[ axis ] )
				return false;
			continue;
		}

		const r32 inv = r32( 1.0 ) / dirs[ axis ];
		r32 tNear = ( lo[ axis ] - origins[ axis ] ) * inv;
		r32 tFar = ( hi[ axis ] - origins[ axis ] ) * inv;
		if ( tNear > tFar )
		{
			const r32 tmp = tNear;
			tNear = tFar;
			tFar = tmp;
		}

		t0 = rpcMax( t0, tNear );
		t1 = rpcMin( t1, tFar );
		if ( t0 > t1 )
			return false;
	}

	*tEnter = t0;
	*tExit = t1;
	return true;
}

}  // namespace

//--------------------------------------------------------------------------------------------------
rpcStaticGrid::rpcStaticGrid( r32 cellSize )
	: m_cellSize( cellSize > r32( 0.5 ) ? cellSize : r32( 0.5 ) )
	, m_effectiveCell( cellSize > r32( 0.5 ) ? cellSize : r32( 0.5 ) )
	, m_stampGen( 0 )
	, m_cellsX( 0 )
	, m_cellsZ( 0 )
	, m_dirty( true )
{
	rpcIdentity( m_bounds.min );
	rpcIdentity( m_bounds.max );
}

//--------------------------------------------------------------------------------------------------
void rpcStaticGrid::Add( rpcBox* box, const rpcAABB& fatAabb )
{
	m_source.push_back( { box, fatAabb } );
	m_dirty = true;
}

//--------------------------------------------------------------------------------------------------
void rpcStaticGrid::Remove( rpcBox* box )
{
	for ( size_t i = 0; i < m_source.size( ); ++i )
	{
		if ( m_source[ i ].box == box )
		{
			m_source[ i ] = m_source.back( );
			m_source.pop_back( );
			m_dirty = true;
			return;
		}
	}
}

//--------------------------------------------------------------------------------------------------
void rpcStaticGrid::Update( rpcBox* box, const rpcAABB& fatAabb )
{
	for ( Item& item : m_source )
	{
		if ( item.box == box )
		{
			item.aabb = fatAabb;
			m_dirty = true;
			return;
		}
	}
}

//--------------------------------------------------------------------------------------------------
void rpcStaticGrid::Clear( )
{
	m_source.clear( );
	m_cellStart.clear( );
	m_items.clear( );
	m_stamp.clear( );
	m_dirty = true;
}

//--------------------------------------------------------------------------------------------------
i32 rpcStaticGrid::CellX( r32 x ) const
{
	const i32 cell = (i32)std::floor( ( x - m_bounds.min.x ) / m_effectiveCell );
	return cell < 0 ? 0 : ( cell >= m_cellsX ? m_cellsX - 1 : cell );
}

//--------------------------------------------------------------------------------------------------
i32 rpcStaticGrid::CellZ( r32 z ) const
{
	const i32 cell = (i32)std::floor( ( z - m_bounds.min.z ) / m_effectiveCell );
	return cell < 0 ? 0 : ( cell >= m_cellsZ ? m_cellsZ - 1 : cell );
}

//--------------------------------------------------------------------------------------------------
i32 rpcStaticGrid::MaxCandidatesPerCell( ) const
{
	if ( m_dirty )
		Rebuild( );

	i32 best = 0;
	for ( i32 c = 0; c < m_cellsX * m_cellsZ; ++c )
		best = rpcMax( best, m_cellStart[ (size_t)c + 1 ] - m_cellStart[ (size_t)c ] );
	return best;
}

//--------------------------------------------------------------------------------------------------
// Counting sort into CSR: one pass to size the cells, one pass to fill them.
void rpcStaticGrid::Rebuild( ) const
{
	m_dirty = false;
	m_cellStart.assign( 1, 0 );
	m_items.clear( );
	m_stamp.clear( );

	if ( m_source.empty( ) )
	{
		m_cellsX = 0;
		m_cellsZ = 0;
		return;
	}

	rpcAABB bounds = m_source.front( ).aabb;
	for ( const Item& item : m_source )
	{
		bounds.min.x = rpcMin( bounds.min.x, item.aabb.min.x );
		bounds.min.z = rpcMin( bounds.min.z, item.aabb.min.z );
		bounds.max.x = rpcMax( bounds.max.x, item.aabb.max.x );
		bounds.max.z = rpcMax( bounds.max.z, item.aabb.max.z );
	}
	// A tiny margin keeps boxes on the boundary inside the grid.
	bounds.min.x -= m_cellSize;
	bounds.min.z -= m_cellSize;
	bounds.max.x += m_cellSize;
	bounds.max.z += m_cellSize;
	m_bounds = bounds;

	// Cap the cell COUNT by growing the cell instead of clamping the index:
	// clamping would fold everything past 1024 cells into the last cell, which
	// silently turns the grid into a linear list for large tracks.
	const r32 extentX = bounds.max.x - bounds.min.x;
	const r32 extentZ = bounds.max.z - bounds.min.z;
	m_effectiveCell = rpcMax( m_cellSize, rpcMax( extentX, extentZ ) / r32( 1024.0 ) );

	m_cellsX = rpcMax( 1, (i32)std::floor( extentX / m_effectiveCell ) + 1 );
	m_cellsZ = rpcMax( 1, (i32)std::floor( extentZ / m_effectiveCell ) + 1 );

	const size_t cells = (size_t)m_cellsX * (size_t)m_cellsZ;
	std::vector<i32> counts( cells + 1, 0 );
	m_cellStart.assign( cells + 1, 0 );

	// Pass 1: count.
	for ( const Item& item : m_source )
	{
		const i32 x0 = CellX( item.aabb.min.x );
		const i32 x1 = CellX( item.aabb.max.x );
		const i32 z0 = CellZ( item.aabb.min.z );
		const i32 z1 = CellZ( item.aabb.max.z );
		for ( i32 z = z0; z <= z1; ++z )
			for ( i32 x = x0; x <= x1; ++x )
				++counts[ (size_t)z * (size_t)m_cellsX + (size_t)x + 1 ];
	}

	// Prefix sum.
	for ( size_t c = 0; c < cells; ++c )
		m_cellStart[ c + 1 ] = m_cellStart[ c ] + counts[ c + 1 ];

	m_items.resize( (size_t)m_cellStart[ cells ] );
	m_stamp.assign( m_items.size( ), 0 );
	m_stampGen = 0;

	// Pass 2: scatter (using a cursor copy of the offsets).
	std::vector<i32> cursor( m_cellStart.begin( ), m_cellStart.end( ) - 1 );
	for ( size_t i = 0; i < m_source.size( ); ++i )
	{
		const Item& item = m_source[ i ];
		const i32 x0 = CellX( item.aabb.min.x );
		const i32 x1 = CellX( item.aabb.max.x );
		const i32 z0 = CellZ( item.aabb.min.z );
		const i32 z1 = CellZ( item.aabb.max.z );
		for ( i32 z = z0; z <= z1; ++z )
			for ( i32 x = x0; x <= x1; ++x )
			{
				const size_t cell = (size_t)z * (size_t)m_cellsX + (size_t)x;
				m_items[ (size_t)cursor[ cell ]++ ] = (i32)i;
			}
	}
}

//--------------------------------------------------------------------------------------------------
// 2D DDA over the XZ projection of the segment, then an exact-conservative
// per-candidate test. Vertical rays (degenerate XZ extent) visit one cell.
void rpcStaticGrid::Raycast( rpcQueryCallback* cb, rpcRaycastData& ray, const rpcBody* ignore ) const
{
	if ( m_dirty )
		Rebuild( );

	if ( m_cellsX == 0 || m_items.empty( ) )
		return;
	[[assume( m_cellsZ > 0 )]];

	const rpcVec3 p0 = ray.start;
	const rpcVec3 p1 = p0 + ray.dir * ray.t;

	r32 tEnter = r32( 0.0 );
	r32 tExit = r32( 1.0 );
	if ( !ClipSegmentXz( p0, p1, m_bounds.min.x, m_bounds.min.z, m_bounds.max.x, m_bounds.max.z,
	                     &tEnter, &tExit ) )
		return;

	const rpcVec3 a = p0 + ( p1 - p0 ) * tEnter;
	const rpcVec3 b = p0 + ( p1 - p0 ) * tExit;

	// Dedupe: boxes spanning several cells are reported once per query.
	if ( ++m_stampGen == 0 )
	{
		std::fill( m_stamp.begin( ), m_stamp.end( ), 0 );
		m_stampGen = 1;
	}

	const auto testCell = [ & ]( i32 cx, i32 cz ) {
		const size_t cell = (size_t)cz * (size_t)m_cellsX + (size_t)cx;
		const i32 first = m_cellStart[ cell ];
		const i32 last = m_cellStart[ cell + 1 ];
		for ( i32 i = first; i < last; ++i )
		{
			const i32 item = m_items[ (size_t)i ];
			if ( m_stamp[ (size_t)item ] == m_stampGen )
				continue;
			m_stamp[ (size_t)item ] = m_stampGen;

			rpcBox* box = m_source[ (size_t)item ].box;
			if ( box->body == ignore )
				continue;
			const rpcAABB& fat = m_source[ (size_t)item ].aabb;
			if ( !SegmentMeetsAabb( fat, p0, p1 ) )
				continue;

			if ( box->Raycast( box->body->GetTransform( ), &ray ) )
			{
				if ( !cb->ReportShape( box ) )
					return;
			}
		}
	};

	i32 cx = CellX( a.x );
	i32 cz = CellZ( a.z );
	const i32 endX = CellX( b.x );
	const i32 endZ = CellZ( b.z );

	const r32 dx = b.x - a.x;
	const r32 dz = b.z - a.z;

	const i32 stepX = dx > r32( 0.0 ) ? 1 : ( dx < r32( 0.0 ) ? -1 : 0 );
	const i32 stepZ = dz > r32( 0.0 ) ? 1 : ( dz < r32( 0.0 ) ? -1 : 0 );

	const r32 invDx = stepX != 0 ? r32( 1.0 ) / ( dx != r32( 0.0 ) ? dx : r32( 1.0 ) ) : r32( 0.0 );
	const r32 invDz = stepZ != 0 ? r32( 1.0 ) / ( dz != r32( 0.0 ) ? dz : r32( 1.0 ) ) : r32( 0.0 );

	// Parametric distance to the next grid plane along each axis.
	const r32 nextX = m_bounds.min.x + (r32)( cx + ( stepX > 0 ? 1 : 0 ) ) * m_effectiveCell;
	const r32 nextZ = m_bounds.min.z + (r32)( cz + ( stepZ > 0 ? 1 : 0 ) ) * m_effectiveCell;
	r32 tMaxX = stepX != 0 ? ( nextX - a.x ) * invDx : RPC_R32_MAX;
	r32 tMaxZ = stepZ != 0 ? ( nextZ - a.z ) * invDz : RPC_R32_MAX;
	const r32 tDeltaX = stepX != 0 ? m_effectiveCell * rpcAbs( invDx ) : RPC_R32_MAX;
	const r32 tDeltaZ = stepZ != 0 ? m_effectiveCell * rpcAbs( invDz ) : RPC_R32_MAX;

	// Guard against pathological grids: a step budget keeps the walk bounded.
	i32 budget = m_cellsX + m_cellsZ + 8;

	while ( budget-- > 0 )
	{
		testCell( cx, cz );

		if ( cx == endX && cz == endZ )
			break;

		if ( tMaxX < tMaxZ )
		{
			if ( cx == endX )
				break;
			cx += stepX;
			tMaxX += tDeltaX;
		}
		else
		{
			if ( cz == endZ )
				break;
			cz += stepZ;
			tMaxZ += tDeltaZ;
		}

		if ( cx < 0 || cx >= m_cellsX || cz < 0 || cz >= m_cellsZ )
			break;
	}
}
