//--------------------------------------------------------------------------------------------------
/**
@file	rpcBroadPhase.h

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

#ifndef RPCBROADPHASE_H
#define RPCBROADPHASE_H

#include "../common/rpcTypes.h"
#include "rpcDynamicAABBTree.h"
#include "../common/rpcMemory.h"

//--------------------------------------------------------------------------------------------------
// rpcBroadPhase
//--------------------------------------------------------------------------------------------------
class rpcContactManager;
struct rpcBox;
struct rpcTransform;
struct rpcAABB;

struct rpcContactPair
{
	i32 A;
	i32 B;
};

class rpcBroadPhase
{
public:
	rpcBroadPhase( rpcContactManager *manager );
	~rpcBroadPhase( );

	void InsertBox( rpcBox *shape, const rpcAABB& aabb );
	void RemoveBox( const rpcBox *shape );

	// Generates the contact list. All previous contacts are returned to the allocator
	// before generation occurs.
	void UpdatePairs( void );

	// Returns true when the proxy actually moved (its fat AABB was left), which
	// is also the only case that cheapens/enables a ray-index update.
	bool Update( i32 id, const rpcAABB& aabb );

	bool TestOverlap( i32 A, i32 B ) const;

	// Per-step work counters (O(1), always maintained — they size the broadphase
	// cost: proxies re-queried, candidate pairs produced, pairs after dedupe).
	i32 MoveCount( ) const { return m_lastMoveCount; }
	i32 PairCount( ) const { return m_pairCount; }
	i32 UniquePairCount( ) const { return m_uniquePairCount; }

	// Sub-stage timing of the last UpdatePairs: sort + dedupe + contact creation,
	// versus the tree queries that produced the candidates. Off by default.
	void SetTiming( bool on ) { m_timing = on; }
	double SortMs( ) const { return m_sortMs; }

private:
	rpcContactManager *m_manager;

	rpcContactPair* m_pairBuffer;
	i32 m_pairCount;
	i32 m_pairCapacity;
	i32 m_uniquePairCount;   // last UpdatePairs: pairs left after dedupe
	bool m_timing;
	double m_sortMs;

	i32* m_moveBuffer;
	i32 m_moveCount;
	i32 m_lastMoveCount;   // moves consumed by the last UpdatePairs
	i32 m_moveCapacity;

	rpcDynamicAABBTree m_tree;
	i32 m_currentIndex;

	void BufferMove( i32 id );
	bool TreeCallBack( i32 index );

	friend class rpcDynamicAABBTree;
	friend class rpcScene;
};

inline bool rpcBroadPhase::TreeCallBack( i32 index )
{
	// Cannot collide with self
	if ( index == m_currentIndex )
		return true;

	if ( m_pairCount == m_pairCapacity )
	{
		rpcContactPair* oldBuffer = m_pairBuffer;
		m_pairCapacity *= 2;
		m_pairBuffer = (rpcContactPair*)rpcAlloc( m_pairCapacity * sizeof( rpcContactPair ) );
		memcpy( m_pairBuffer, oldBuffer, m_pairCount * sizeof( rpcContactPair ) );
		rpcFree( oldBuffer );
	}

	i32 iA = rpcMin( index, m_currentIndex );
	i32 iB = rpcMax( index, m_currentIndex );

	m_pairBuffer[ m_pairCount ].A = iA;
	m_pairBuffer[ m_pairCount ].B = iB;
	++m_pairCount;

	return true;
}

#endif // RPCBROADPHASE_H
