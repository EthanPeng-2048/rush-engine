//--------------------------------------------------------------------------------------------------
// rpcDynamicAABBTree.inl
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
// rpcDynamicAABBTree
//--------------------------------------------------------------------------------------------------
inline void rpcDynamicAABBTree::AddToFreeList( i32 index )
{
	for ( i32 i = index; i < m_capacity - 1; ++i)
	{
		m_nodes[ i ].next = i + 1;
		m_nodes[ i ].height = Node::Null;
	}

	m_nodes[ m_capacity - 1 ].next = Node::Null;
	m_nodes[ m_capacity - 1 ].height = Node::Null;
	m_freeList = index;
}

//--------------------------------------------------------------------------------------------------
inline void rpcDynamicAABBTree::DeallocateNode( i32 index )
{
	assert( index >= 0 && index < m_capacity );

	m_nodes[ index ].next = m_freeList;
	m_nodes[ index ].height = Node::Null;
	m_freeList = index;

	--m_count;
}

//--------------------------------------------------------------------------------------------------
template <typename T>
inline void rpcDynamicAABBTree::Query( T *cb, const rpcAABB& aabb ) const
{
	const i32 k_stackCapacity = 256;
	i32 stack[ k_stackCapacity ];
	i32 sp = 1;

	*stack = m_root;

	while ( sp )
	{
		// k_stackCapacity too small
		assert( sp < k_stackCapacity );

		i32 id = stack[ --sp ];

		const Node *n = m_nodes + id;
		++m_queryNodes;
		++m_queryNodes;

		if ( rpcAABBtoAABB( aabb, n->aabb ) )
		{
			if ( n->IsLeaf( ) )
			{
				++m_queryLeaves;

				if ( !cb->TreeCallBack( id ) )
					return;
			}
			else
			{
				stack[ sp++ ] = n->left;
				stack[ sp++ ] = n->right;
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------
template <typename T>
void rpcDynamicAABBTree::Query( T *cb, rpcRaycastData& rayCast ) const
{
	const r32 k_epsilon = r32( 1.0e-6 );
	const i32 k_stackCapacity = 256;
	i32 stack[ k_stackCapacity ];
	i32 sp = 1;

	*stack = m_root;

	rpcVec3 p0 = rayCast.start;
	rpcVec3 p1 = p0 + rayCast.dir * rayCast.t;

	while( sp )
	{
		// k_stackCapacity too small
		assert( sp < k_stackCapacity );

		i32 id = stack[--sp];

		if ( id == Node::Null )
			continue;

		const Node *n = m_nodes + id;
		++m_queryNodes;

		rpcVec3 e = n->aabb.max - n->aabb.min;
		rpcVec3 d = p1 - p0;
		rpcVec3 m = p0 + p1 - n->aabb.min - n->aabb.max;

		r32 adx = rpcAbs( d.x );

		if ( rpcAbs( m.x ) > e.x + adx )
			continue;

		r32 ady = rpcAbs( d.y );

		if ( rpcAbs( m.y ) > e.y + ady )
			continue;

		r32 adz = rpcAbs( d.z );

		if ( rpcAbs( m.z ) > e.z + adz )
			continue;

		adx += k_epsilon;
		ady += k_epsilon;
		adz += k_epsilon;

		if( rpcAbs( m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady )
			continue;

		if( rpcAbs( m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx )
			continue;

		if ( rpcAbs( m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx )
			continue;

		if ( n->IsLeaf( ) )
		{
			++m_queryLeaves;

			if ( !cb->TreeCallBack( id ) )
				return;
		}

		else
		{
			stack[ sp++ ] = n->left;
			stack[ sp++ ] = n->right;
		}
	}
}

//--------------------------------------------------------------------------------------------------
template <typename T>
void rpcDynamicAABBTree::QueryBatch( T *cb, rpcRaycastData* rayCasts, i32 count ) const
{
	assert( count > 0 && count <= 32 );

	const r32 k_epsilon = r32( 1.0e-6 );
	const i32 k_stackCapacity = 256;
	i32 stackId[ k_stackCapacity ];
	u32 stackMask[ k_stackCapacity ];
	i32 sp = 1;

	stackId[ 0 ] = m_root;
	stackMask[ 0 ] = 0xFFFFFFFFu;

	// Endpoints at each ray's current t (per-ray closest-hit prune). The
	// traversal never re-reads t, so visited nodes are a superset of the
	// per-ray pruned walk — the box test in the leaf callback enforces t.
	rpcVec3 p0[ 32 ];
	rpcVec3 p1[ 32 ];
	for ( i32 j = 0; j < count; ++j )
	{
		p0[ j ] = rayCasts[ j ].start;
		p1[ j ] = p0[ j ] + rayCasts[ j ].dir * rayCasts[ j ].t;
	}

	while ( sp )
	{
		// k_stackCapacity too small
		assert( sp < k_stackCapacity );

		--sp;
		const i32 id = stackId[ sp ];
		const u32 mask = stackMask[ sp ];

		if ( id == Node::Null )
			continue;

		const Node *n = m_nodes + id;
		++m_queryNodes;

		u32 childMask = 0;
		for ( i32 j = 0; j < count; ++j )
		{
			if ( !( mask & ( u32( 1 ) << j ) ) )
				continue;

			const rpcVec3 e = n->aabb.max - n->aabb.min;
			const rpcVec3 d = p1[ j ] - p0[ j ];
			const rpcVec3 c = p0[ j ] + p1[ j ] - n->aabb.min - n->aabb.max;

			const r32 adx = rpcAbs( d.x );
			if ( rpcAbs( c.x ) > e.x + adx )
				continue;

			const r32 ady = rpcAbs( d.y );
			if ( rpcAbs( c.y ) > e.y + ady )
				continue;

			const r32 adz = rpcAbs( d.z );
			if ( rpcAbs( c.z ) > e.z + adz )
				continue;

			// The same slab test as the single-ray Query, epsilon included.
			if ( rpcAbs( c.y * d.z - c.z * d.y ) > e.y * ( adz + k_epsilon ) + e.z * ( ady + k_epsilon ) )
				continue;

			if ( rpcAbs( c.z * d.x - c.x * d.z ) > e.x * ( adz + k_epsilon ) + e.z * ( adx + k_epsilon ) )
				continue;

			if ( rpcAbs( c.x * d.y - c.y * d.x ) > e.x * ( ady + k_epsilon ) + e.y * ( adx + k_epsilon ) )
				continue;

			childMask |= u32( 1 ) << j;
		}

		if ( childMask == 0 )
			continue;

		if ( n->IsLeaf( ) )
		{
			for ( i32 j = 0; j < count; ++j )
			{
				if ( !( childMask & ( u32( 1 ) << j ) ) )
					continue;
				if ( !cb->TreeCallBack( id, j ) )
					return;
			}
		}
		else
		{
			stackId[ sp ] = n->left;
			stackMask[ sp ] = childMask;
			++sp;
			stackId[ sp ] = n->right;
			stackMask[ sp ] = childMask;
			++sp;
		}
	}
}
