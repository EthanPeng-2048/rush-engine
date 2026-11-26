//--------------------------------------------------------------------------------------------------
/**
@file	rpcBroadPhase.cpp

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

#include "rpcBroadPhase.h"
#include "../collision/rpcBox.h"
#include "../common/rpcGeometry.h"
#include "../dynamics/rpcContactManager.h"

//--------------------------------------------------------------------------------------------------
// rpcBroadPhase
//--------------------------------------------------------------------------------------------------
rpcBroadPhase::rpcBroadPhase( rpcContactManager *manager )
{
	m_manager = manager;

	m_pairCount = 0;

	m_moveCount = 0;
	m_lastMoveCount = 0;
	m_moveCapacity = 64;
	m_moveBuffer = (i32*)rpcAlloc( m_moveCapacity * sizeof( i32 ) );
}

//--------------------------------------------------------------------------------------------------
rpcBroadPhase::~rpcBroadPhase( )
{
	rpcFree( m_moveBuffer );
}

//--------------------------------------------------------------------------------------------------
void rpcBroadPhase::InsertBox( rpcBox *box, const rpcAABB& aabb )
{
	i32 id = m_tree.Insert( aabb, box );
	box->broadPhaseIndex = id;
	BufferMove( id );
}

//--------------------------------------------------------------------------------------------------
void rpcBroadPhase::RemoveBox( const rpcBox *box )
{
	m_tree.Remove( box->broadPhaseIndex );
}

//--------------------------------------------------------------------------------------------------
void rpcBroadPhase::UpdatePairs( )
{
	m_pairCount = 0;
	// Report the buffer that is about to be consumed, not the emptied one.
	m_lastMoveCount = m_moveCount;

	// Query the tree with all moving boxes. Each overlapping proxy is offered to
	// the manager immediately: the manager keeps a hash of live shape pairs, so
	// duplicates (two moved proxies finding each other) cost one probe instead
	// of a sort plus a walk of the body's contact edges.
	for ( i32 i = 0; i < m_moveCount; ++i)
	{
		m_currentIndex = m_moveBuffer[ i ];
		rpcAABB aabb = m_tree.GetFatAABB( m_currentIndex );

		m_tree.Query( this, aabb );
	}

	// Reset the move buffer
	m_moveCount = 0;

	// Debug-only tree integrity check (walks the free list — keep out of Release)
#ifndef NDEBUG
	m_tree.Validate( );
#endif
}

//--------------------------------------------------------------------------------------------------
// Tree query callback: one overlapping proxy. The manager de-duplicates in O(1),
// so the pair is offered immediately instead of being buffered and sorted.
bool rpcBroadPhase::TreeCallBack( i32 index )
{
	// Cannot collide with self
	if ( index == m_currentIndex )
		return true;

	++m_pairCount;

	rpcBox* A = (rpcBox*)m_tree.GetUserData( m_currentIndex );
	rpcBox* B = (rpcBox*)m_tree.GetUserData( index );
	m_manager->AddContact( A, B );

	return true;
}

//--------------------------------------------------------------------------------------------------
bool rpcBroadPhase::Update( i32 id, const rpcAABB& aabb )
{
	if ( !m_tree.Update( id, aabb ) )
		return false;

	BufferMove( id );
	return true;
}

//--------------------------------------------------------------------------------------------------
bool rpcBroadPhase::TestOverlap( i32 A, i32 B ) const
{
	return rpcAABBtoAABB( m_tree.GetFatAABB( A ), m_tree.GetFatAABB( B ) );
}

//--------------------------------------------------------------------------------------------------
void rpcBroadPhase::BufferMove( i32 id )
{
	if ( m_moveCount == m_moveCapacity )
	{
		i32* oldBuffer = m_moveBuffer;
		m_moveCapacity *= 2;
		m_moveBuffer = (i32*)rpcAlloc( m_moveCapacity * sizeof( i32 ) );
		memcpy( m_moveBuffer, oldBuffer, m_moveCount * sizeof( i32 ) );
		rpcFree( oldBuffer );
	}

	m_moveBuffer[ m_moveCount++ ] = id;
}
