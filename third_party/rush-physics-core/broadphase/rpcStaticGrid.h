//--------------------------------------------------------------------------------------------------
/**
@file	rpcStaticGrid.h

	Uniform grid over static collision geometry (implementation of rpcRayIndex).

	Static track geometry never moves, so a flat grid beats a general BVH for
	queries that are local (a suspension ray is ~0.7 m long): the cell walk is
	O(cells touched) with sequential memory, no pointer chasing, and the whole
	accelerator is rebuilt only when geometry is added.

	Layout: cells are a CSR pair (cellStart offsets + contiguous item array),
	built by counting sort so a query reads items in ascending index order. Each
	item carries a visit stamp so boxes spanning several cells are reported once.
*/
//--------------------------------------------------------------------------------------------------

#ifndef RPCSTATICGRID_H
#define RPCSTATICGRID_H

#include <vector>

#include "rpcRayIndex.h"
#include "../common/rpcGeometry.h"

//--------------------------------------------------------------------------------------------------
// rpcStaticGrid
//--------------------------------------------------------------------------------------------------
class rpcStaticGrid final : public rpcRayIndex
{
public:
	explicit rpcStaticGrid( r32 cellSize );

	// rpcRayIndex
	const char* Name( ) const override { return "static-grid"; }
	void Add( rpcBox* box, const rpcAABB& fatAabb ) override;
	void Remove( rpcBox* box ) override;
	void Update( rpcBox* box, const rpcAABB& fatAabb ) override;
	void Clear( ) override;
	void Raycast( rpcQueryCallback* cb, rpcRaycastData& ray, const rpcBody* ignore = nullptr ) const override;

	// Diagnostics (bench / tests): cell count and peak candidates per cell.
	i32 CellCount( ) const override { return m_cellsX * m_cellsZ; }
	i32 ItemCount( ) const { return (i32)m_items.size( ); }
	i32 MaxCandidatesPerCell( ) const;

private:
	struct Item
	{
		rpcBox* box;
		rpcAABB aabb;
	};

	void Rebuild( ) const;
	i32 CellX( r32 x ) const;
	i32 CellZ( r32 z ) const;

	r32 m_cellSize;
	mutable r32 m_effectiveCell;   // >= m_cellSize; derived from the bounds
	mutable std::vector<Item> m_source;   // insertion order; rebuilt on demand
	mutable std::vector<i32> m_cellStart; // size cells + 1 (CSR)
	mutable std::vector<i32> m_items;     // item indices grouped by cell
	mutable std::vector<i32> m_stamp;     // per-item visit stamp (dedupe)
	mutable i32 m_stampGen;
	mutable i32 m_cellsX;
	mutable i32 m_cellsZ;
	mutable rpcAABB m_bounds;
	mutable bool m_dirty;
};

#endif // RPCSTATICGRID_H
