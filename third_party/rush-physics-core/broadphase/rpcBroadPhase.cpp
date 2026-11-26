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

#include <algorithm>
#include <chrono>

//--------------------------------------------------------------------------------------------------
// rpcBroadPhase
//--------------------------------------------------------------------------------------------------
rpcBroadPhase::rpcBroadPhase( rpcContactManager *manager )
{
	m_manager = manager;

	m_pairCount = 0;
	m_pairCapacity = 64;
	m_pairBuffer = (rpcContactPair*)rpcAlloc( m_pairCapacity * sizeof( rpcContactPair ) );

	m_moveCount = 0;
	m_lastMoveCount = 0;
	m_moveCapacity = 64;
	m_moveBuffer = (i32*)rpcAlloc( m_moveCapacity * sizeof( i32 ) );

	m_timing = false;
	m_sortMs = 0.0;
}

//--------------------------------------------------------------------------------------------------
rpcBroadPhase::~rpcBroadPhase( )
{
	rpcFree( m_moveBuffer );
	rpcFree( m_pairBuffer );
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
inline bool ContactPairSort( const rpcContactPair& lhs, const rpcContactPair& rhs )
{
	if ( lhs.A < rhs.A )
		return true;

	if ( lhs.A == rhs.A )
		return lhs.B < rhs.B;

	return false;
}

//--------------------------------------------------------------------------------------------------
void rpcBroadPhase::UpdatePairs( )
{
	m_pairCount = 0;
	m_uniquePairCount = 0;
	// Report the buffer that is about to be consumed, not the emptied one.
	m_lastMoveCount = m_moveCount;

	// Query the tree with all moving boxs
	for ( i32 i = 0; i < m_moveCount; ++i)
	{
		m_currentIndex = m_moveBuffer[ i ];
		rpcAABB aabb = m_tree.GetFatAABB( m_currentIndex );

		// @TODO: Use a static and non-static tree and query one against the other.
		//        This will potentially prevent (gotta think about this more) time
		//        wasted with queries of static bodies against static bodies, and
		//        kinematic to kinematic.
		m_tree.Query( this, aabb );
	}

	// Reset the move buffer
	m_moveCount = 0;

	const auto sortStart = m_timing ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

	// Sort pairs to expose duplicates
	std::sort( m_pairBuffer, m_pairBuffer + m_pairCount, ContactPairSort );

	// Queue manifolds for solving
	{
		i32 i = 0;
		while ( i < m_pairCount )
		{
			// Add contact to manager
			rpcContactPair* pair = m_pairBuffer + i;
			rpcBox *A = (rpcBox*)m_tree.GetUserData( pair->A );
			rpcBox *B = (rpcBox*)m_tree.GetUserData( pair->B );
			m_manager->AddContact( A, B );
			++m_uniquePairCount;

			++i;

			// Skip duplicate pairs by iterating i until we find a unique pair
			while ( i < m_pairCount )
			{
				rpcContactPair* potentialDup = m_pairBuffer + i;

				if ( pair->A != potentialDup->A || pair->B != potentialDup->B )
					break;

				++i;
			}
		}
	}

	// Debug-only tree integrity check (walks the free list — keep out of Release)
#ifndef NDEBUG
	m_tree.Validate( );
#endif

	if ( m_timing )
		m_sortMs = std::chrono::duration<double, std::milli>(
		               std::chrono::steady_clock::now( ) - sortStart ).count( );
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
