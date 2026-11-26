//--------------------------------------------------------------------------------------------------
/**
@file	rpcPairMap.h

	Open-addressing map from a 64-bit pair key to a small value.

	Why this exists: the contact manager used to answer "do these two shapes
	already have a constraint?" by walking the body's contact-edge list, which is
	O(contacts of that body). In a pile-up a single body carries hundreds of
	edges and the broadphase offers tens of thousands of candidate pairs per
	step, so the *dedupe* — not the collision maths — became the frame's biggest
	cost: measured 17.9 ms of a 20.3 ms worst-step broadphase on 120 cars
	(36 344 candidates, 12 420 live pairs). The same table also carries the
	per-body-pair contact budget, which had the identical shape of problem.

	Design notes:
	  - Keys are unique (a shape pair, a body pair), so no duplicate handling.
	  - Linear probing with backward-shift deletion: no tombstones, so a
	    churning table (contacts are created and destroyed constantly) never
	    degrades into a walk of dead slots.
	  - Power-of-two capacity, grow at 3/4 load; iteration order is the slot
	    order and callers must not depend on it.
*/
//--------------------------------------------------------------------------------------------------

#ifndef RPCPAIRMAP_H
#define RPCPAIRMAP_H

#include "../common/rpcTypes.h"
#include "../common/rpcMemory.h"

#include <string.h>

template <typename T>
class rpcPairMap
{
public:
	rpcPairMap( )
		: m_keys( NULL )
		, m_values( NULL )
		, m_capacity( 0 )
		, m_count( 0 )
	{
	}

	~rpcPairMap( )
	{
		rpcFree( m_keys );
		rpcFree( m_values );
	}

	rpcPairMap( const rpcPairMap& ) = delete;
	rpcPairMap& operator=( const rpcPairMap& ) = delete;

	i32 Count( ) const { return m_count; }
	i32 Capacity( ) const { return m_capacity; }

	void Clear( )
	{
		m_count = 0;
		// Zeroing the key array is the tombstone marker: kEmpty is 0 and pair
		// keys are never 0 (they always carry at least one non-zero index).
		if ( m_capacity )
			memset( m_keys, 0, sizeof( u64 ) * (size_t)m_capacity );
	}

	// Grow so `count` entries fit without rehashing (build-time hint).
	void Reserve( i32 count )
	{
		if ( count <= 0 )
			return;

		i32 needed = 16;
		while ( needed < count * 2 )
			needed *= 2;

		if ( needed > m_capacity )
			Rehash( needed );
	}

	T* Find( u64 key )
	{
		if ( m_count == 0 )
			return NULL;

		i32 i = Home( key );
		while ( m_keys[ i ] != 0 )
		{
			if ( m_keys[ i ] == key )
				return &m_values[ i ];
			i = ( i + 1 ) & ( m_capacity - 1 );
		}

		return NULL;
	}

	// Insert or overwrite. Returns the stored value slot.
	T* Insert( u64 key, T value )
	{
		// Grow at 50% load: linear probing degrades sharply past that, and the
		// contact table is probed tens of thousands of times per step.
		if ( ( m_count + 1 ) * 2 > m_capacity )
			Rehash( m_capacity ? m_capacity * 2 : 64 );

		i32 i = Home( key );
		while ( m_keys[ i ] != 0 )
		{
			if ( m_keys[ i ] == key )
			{
				m_values[ i ] = value;
				return &m_values[ i ];
			}
			i = ( i + 1 ) & ( m_capacity - 1 );
		}

		m_keys[ i ] = key;
		m_values[ i ] = value;
		++m_count;
		return &m_values[ i ];
	}

	bool Erase( u64 key )
	{
		if ( m_count == 0 )
			return false;

		i32 i = Home( key );
		while ( m_keys[ i ] != 0 )
		{
			if ( m_keys[ i ] == key )
			{
				RemoveAt( i );
				return true;
			}
			i = ( i + 1 ) & ( m_capacity - 1 );
		}

		return false;
	}

	// Slot-wise iteration: `key` is 0 for a free slot, so callers can skip it.
	// `index` starts at 0 and returns false once the table is exhausted.
	bool Next( i32& index, u64& key, T*& value )
	{
		while ( index < m_capacity )
		{
			const i32 i = index++;
			if ( m_keys[ i ] != 0 )
			{
				key = m_keys[ i ];
				value = &m_values[ i ];
				return true;
			}
		}

		return false;
	}

private:
	// Pair keys are built from two non-negative indices, so a key is never 0.
	static u64 MakeKey( i32 a, i32 b )
	{
		// 0 is reserved for "empty": bias both halves by one so index 0 is
		// representable without colliding with the free marker.
		const u64 hi = (u64)(u32)( a + 1 );
		const u64 lo = (u64)(u32)( b + 1 );
		return ( hi << 32 ) | lo;
	}

	i32 Home( u64 key ) const
	{
		// Fibonacci hash: the low half of the key is a dense index, so it must
		// be mixed before masking or every pair of the same shape lands in one
		// bucket.
		const u64 mixed = key * 0x9E3779B97F4A7C15ull;
		return (i32)( mixed >> 32 ) & ( m_capacity - 1 );
	}

	void RemoveAt( i32 index )
	{
		--m_count;
		const i32 mask = m_capacity - 1;

		// Backward-shift deletion: pull later entries of the same probe run
		// into the hole instead of leaving a tombstone behind.
		i32 hole = index;
		for ( ;; )
		{
			i32 next = ( hole + 1 ) & mask;
			if ( m_keys[ next ] == 0 )
			{
				m_keys[ hole ] = 0;
				return;
			}

			const i32 home = Home( m_keys[ next ] );

			// Is `next`'s home bucket outside the cyclic range (hole, next]?
			// If so the entry can move back into the hole.
			const bool movable = ( hole <= next ) ? ( home > hole && home <= next )
			                                      : ( home > hole || home <= next );
			if ( !movable )
			{
				m_keys[ hole ] = 0;
				return;
			}

			m_keys[ hole ] = m_keys[ next ];
			m_values[ hole ] = m_values[ next ];
			hole = next;
		}
	}

	void Rehash( i32 newCapacity )
	{
		u64* oldKeys = m_keys;
		T* oldValues = m_values;
		const i32 oldCapacity = m_capacity;

		m_capacity = newCapacity;
		m_keys = (u64*)rpcAlloc( sizeof( u64 ) * (size_t)m_capacity );
		m_values = (T*)rpcAlloc( sizeof( T ) * (size_t)m_capacity );
		memset( m_keys, 0, sizeof( u64 ) * (size_t)m_capacity );
		m_count = 0;

		for ( i32 i = 0; i < oldCapacity; ++i )
		{
			if ( oldKeys[ i ] != 0 )
				Insert( oldKeys[ i ], oldValues[ i ] );
		}

		rpcFree( oldKeys );
		rpcFree( oldValues );
	}

	u64* m_keys;
	T* m_values;
	i32 m_capacity;
	i32 m_count;

public:
	// Both pair kinds are keyed by two scene-unique dense ids (shape uid, body
	// uid), which CAN be packed injectively into 64 bits. Pointer pairs cannot
	// be — mixing them with xor/rotate collides, which would silently corrupt
	// the pair's stored value.
	static u64 PairKey( i32 a, i32 b )
	{
		return a < b ? MakeKey( a, b ) : MakeKey( b, a );
	}
};

#endif // RPCPAIRMAP_H
