//--------------------------------------------------------------------------------------------------
/**
@file	rpcDynamicAABBTree.h

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

#ifndef RPCDYNAMICAABBTREE_H
#define RPCDYNAMICAABBTREE_H

#include "../math/rpcMath.h"
#include "../common/rpcGeometry.h"

//--------------------------------------------------------------------------------------------------
// rpcDynamicAABBTree
//--------------------------------------------------------------------------------------------------
// Resources:
// http://box2d.org/2014/08/balancing-dynamic-trees/
// http://www.randygaul.net/2013/08/06/dynamic-aabb-tree/
class rpcRender;

class rpcDynamicAABBTree
{
public:
	rpcDynamicAABBTree( );
	~rpcDynamicAABBTree( );

	// Provide tight-AABB
	i32 Insert( const rpcAABB& aabb, void *userData );
	void Remove( i32 id );
	bool Update( i32 id, const rpcAABB& aabb );

	void *GetUserData( i32 id ) const;
	const rpcAABB& GetFatAABB( i32 id ) const;
	void Render( rpcRender *render ) const;

	template <typename T>
	void Query( T *cb, const rpcAABB& aabb ) const;
	template <typename T>
	void Query( T *cb, rpcRaycastData& rayCast ) const;
	// Batch raycast: N rays (<= 32) tested against the tree in ONE traversal.
	// Each ray keeps its own closest-hit state (start/dir/t); the callback is
	// invoked as TreeCallBack( leafId, rayIndex ) per hit. Visits are a
	// superset of N separate Query calls (no per-ray endpoint pruning), so
	// results are identical but the traversal cost is paid once.
	template <typename T>
	void QueryBatch( T *cb, rpcRaycastData* rayCasts, i32 count ) const;

	// For testing
	void Validate( ) const;

	// Traversal counters (accumulated since construction): node visits and leaf
	// callbacks across every query. Used by the broadphase benchmarks to spot
	// tree-shape regressions; the cost is one increment per visited node.
	u64 QueryNodeVisits( ) const { return m_queryNodes; }
	u64 QueryLeafVisits( ) const { return m_queryLeaves; }
	void ResetQueryCounters( ) const { m_queryNodes = 0; m_queryLeaves = 0; }
	// Branch/leaf counts of the live tree (root AABB + height help see how
	// badly the fattening inflates the hierarchy).
	i32 Count( ) const { return m_count; }
	i32 Height( ) const;
	const rpcAABB& RootAABB( ) const { return m_nodes[ m_root ].aabb; }

private:
	struct Node
	{
		bool IsLeaf( void ) const
		{
			// The right leaf does not use the same memory as the userdata,
			// and will always be Null (no children)
			return right == Null;
		}

		// Fat AABB for leafs, bounding AABB for branches
		rpcAABB aabb;

		union
		{
			i32 parent;
			i32 next; // free list
		};

		// Child indices
		struct
		{
			i32 left;
			i32 right;
		};

		// Since only leaf nodes hold userdata, we can use the
		// same memory used for left/right indices to store
		// the userdata void pointer
		void *userData;

		// leaf = 0, free nodes = -1
		i32 height;

		static const i32 Null = -1;
	};

	inline i32 AllocateNode( );
	inline void DeallocateNode( i32 index );
	i32 Balance( i32 index );
	void InsertLeaf( i32 index );
	void RemoveLeaf( i32 index );
	void ValidateStructure( i32 index ) const;
	void RenderNode( rpcRender *render, i32 index ) const;

	// Correct AABB hierarchy heights and AABBs starting at supplied
	// index traversing up the heirarchy
	void SyncHeirarchy( i32 index );

	// Insert nodes at a given index until m_capacity into the free list
	void AddToFreeList( i32 index );

	i32 m_root;
	Node *m_nodes;
	i32 m_count;	// Number of active nodes
	i32 m_capacity;	// Max capacity of nodes
	i32 m_freeList;

	mutable u64 m_queryNodes;
	mutable u64 m_queryLeaves;
};

#include "rpcDynamicAABBTree.inl"

#endif // RPCDYNAMICAABBTREE_H
