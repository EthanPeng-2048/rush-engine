//--------------------------------------------------------------------------------------------------
/**
@file	rpcDynamicTreeRayIndex.h

	Ray index over the dynamic bodies backed by a second, dynamic-only BVH
	(implementation of rpcRayIndex).

	Why a separate tree instead of the broadphase one: the broadphase tree also
	holds every static track shape, so a suspension ray has to descend through
	~13 levels of static nodes to find the road. This index keeps only moving
	shapes (a few thousand leaves at most), so the same query prunes far
	earlier — and it scales with the dynamic population instead of scanning it.

	Insertion/update is driven by the body proxies (rpcScene), the tree's own
	fat-AABB test rejects moves that stay inside the margin, so a cruising car
	costs almost nothing.
*/
//--------------------------------------------------------------------------------------------------

#ifndef RPCDYNAMICTREERAYINDEX_H
#define RPCDYNAMICTREERAYINDEX_H

#include "rpcRayIndex.h"
#include "rpcDynamicAABBTree.h"

//--------------------------------------------------------------------------------------------------
// rpcDynamicTreeRayIndex
//--------------------------------------------------------------------------------------------------
class rpcDynamicTreeRayIndex final : public rpcRayIndex
{
public:
	// rpcRayIndex
	const char* Name( ) const override { return "dynamic-tree"; }
	void Add( rpcBox* box, const rpcAABB& fatAabb ) override;
	void Remove( rpcBox* box ) override;
	void Update( rpcBox* box, const rpcAABB& fatAabb ) override;
	void Clear( ) override;
	void Raycast( rpcQueryCallback* cb, rpcRaycastData& ray, const rpcBody* ignore = nullptr ) const override;

	i32 LeafCount( ) const { return m_tree.Count( ); }

private:
	rpcDynamicAABBTree m_tree;
};

#endif // RPCDYNAMICTREERAYINDEX_H
