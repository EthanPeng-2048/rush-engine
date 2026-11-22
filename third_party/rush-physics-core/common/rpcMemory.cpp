//--------------------------------------------------------------------------------------------------
/**
@file	rpcMemory.cpp

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

#include <cassert>
#include <cstdio>

#include "rpcMemory.h"
#include "../math/rpcMath.h"

//--------------------------------------------------------------------------------------------------
// rpcStack
//--------------------------------------------------------------------------------------------------
rpcStack::rpcStack( )
	: m_memory( 0 )
	, m_entries( (rpcStackEntry*)rpcAlloc( sizeof( rpcStackEntry ) * 64 ) )
	, m_index( 0 )
	, m_allocation( 0 )
	, m_entryCount( 0 )
	, m_entryCapacity( 64 )
	, m_stackSize( 0 )
{
}

//--------------------------------------------------------------------------------------------------
rpcStack::~rpcStack( )
{
	if (m_memory) rpcFree( m_memory );
	assert( m_index == 0 );
	assert( m_entryCount == 0 );
}

//--------------------------------------------------------------------------------------------------
void rpcStack::Reserve( u32 size )
{
	assert( !m_index );

	if ( size == 0 )
		return;

	if ( size >= m_stackSize )
	{
		if ( m_memory ) rpcFree( m_memory );
		m_memory = (u8*)rpcAlloc( size );
		m_stackSize = size;
	}
}

//--------------------------------------------------------------------------------------------------
void *rpcStack::Allocate( i32 size )
{
	assert( m_index + size <= m_stackSize );

	if ( m_entryCount == m_entryCapacity )
	{
		rpcStackEntry* oldEntries = m_entries;
		m_entryCapacity *= 2;
		m_entries = (rpcStackEntry*)rpcAlloc( m_entryCapacity * sizeof( rpcStackEntry ) );
		memcpy( m_entries, oldEntries, m_entryCount * sizeof( rpcStackEntry ) );
		rpcFree( oldEntries );
	}

	rpcStackEntry* entry = m_entries + m_entryCount;
	entry->size = size;

	entry->data = m_memory + m_index;
	m_index += size;

	m_allocation += size;
	++m_entryCount;

	return entry->data;
}

//--------------------------------------------------------------------------------------------------
void rpcStack::Free( void *data )
{
	// Cannot call free when there are no entries.
	assert( m_entryCount > 0 );

	rpcStackEntry *entry = m_entries + m_entryCount - 1;

	// Validate that the data * is a proper location to free.
	// Must be in reverse order of allocation.
	assert( data == entry->data );

	m_index -= entry->size;

	m_allocation -= entry->size;
	--m_entryCount;
}

//--------------------------------------------------------------------------------------------------
// rpcHeap
//--------------------------------------------------------------------------------------------------
rpcHeap::rpcHeap( )
{
	m_memory = (rpcHeader*)rpcAlloc( rpck_heapSize );
	m_memory->next = NULL;
	m_memory->prev = NULL;
	m_memory->size = rpck_heapSize;

	m_freeBlocks = (rpcFreeBlock*)rpcAlloc( sizeof( rpcFreeBlock ) * rpck_heapInitialCapacity );
	m_freeBlockCount = 1;
	m_freeBlockCapacity = rpck_heapInitialCapacity;

	m_freeBlocks->header = m_memory;
	m_freeBlocks->size = rpck_heapSize;
}

//--------------------------------------------------------------------------------------------------
rpcHeap::~rpcHeap( )
{
	rpcFree( m_memory );
	rpcFree( m_freeBlocks );
}

//--------------------------------------------------------------------------------------------------
void *rpcHeap::Allocate( i32 size )
{
	i32 sizeNeeded = size + sizeof( rpcHeader );
	rpcFreeBlock* firstFit = NULL;

	for ( i32 i = 0; i < m_freeBlockCount; ++i )
	{
		rpcFreeBlock* block = m_freeBlocks + i;

		if ( block->size >= sizeNeeded )
		{
			firstFit = block;
			break;
		}
	}

	if ( !firstFit )
		return NULL;

	rpcHeader* node = firstFit->header;
	rpcHeader* newNode = RPC_PTR_ADD( node, sizeNeeded );
	node->size = sizeNeeded;

	firstFit->size -= sizeNeeded;
	firstFit->header = newNode;

	newNode->next = node->next;
	if ( node->next )
		node->next->prev = newNode;
	node->next = newNode;
	newNode->prev = node;

	return RPC_PTR_ADD( node, sizeof( rpcHeader ) );
}

//--------------------------------------------------------------------------------------------------
void rpcHeap::Free( void *memory )
{
	assert( memory );
	rpcHeader* node = (rpcHeader*)RPC_PTR_ADD( memory, -i32( sizeof( rpcHeader ) ) );

	rpcHeader* next = node->next;
	rpcHeader* prev = node->prev;
	rpcFreeBlock* nextBlock = NULL;
	i32 prevBlockIndex = ~0;
	rpcFreeBlock* prevBlock = NULL;
	i32 freeBlockCount = m_freeBlockCount;

	for ( i32 i = 0; i < freeBlockCount; ++i )
	{
		rpcFreeBlock* block = m_freeBlocks + i;
		rpcHeader* header = block->header;

		if ( header == next )
			nextBlock = block;

		else if ( header == prev )
		{
			prevBlock = block;
			prevBlockIndex = i;
		}
	}

	bool merged = false;

	if ( prevBlock )
	{
		merged = true;

		prev->next = next;
		if ( next )
			next->prev = prev;

		prevBlock->size += node->size;
		prev->size = prevBlock->size;

		if ( nextBlock )
		{
			nextBlock->header = prev;
			nextBlock->size += prev->size;
			prev->size = nextBlock->size;

			rpcHeader* nextnext = next->next;
			prev->next = nextnext;

			if ( nextnext )
				nextnext->prev = prev;

			// Remove the nextBlock from the freeBlocks array
			assert( m_freeBlockCount );
			assert( prevBlockIndex != ~0 );
			--m_freeBlockCount;
			m_freeBlocks[ prevBlockIndex ] = m_freeBlocks[ m_freeBlockCount ];
		}
	}

	else if ( nextBlock )
	{
		merged = true;

		nextBlock->header = node;
		nextBlock->size += node->size;
		node->size = nextBlock->size;

		rpcHeader* nextnext = next->next;

		if ( nextnext )
			nextnext->prev = node;

		node->next = nextnext;
	}

	if ( !merged )
	{
		rpcFreeBlock block;
		block.header = node;
		block.size = node->size;

		if ( m_freeBlockCount == m_freeBlockCapacity )
		{
			rpcFreeBlock* oldBlocks = m_freeBlocks;
			i32 oldCapacity = m_freeBlockCapacity;

			m_freeBlockCapacity *= 2;
			m_freeBlocks = (rpcFreeBlock*)rpcAlloc( sizeof( rpcFreeBlock ) * m_freeBlockCapacity );

			memcpy( m_freeBlocks, oldBlocks, sizeof( rpcFreeBlock ) * oldCapacity );
			rpcFree( oldBlocks );
		}

		m_freeBlocks[ m_freeBlockCount++ ] = block;
	}
}

//--------------------------------------------------------------------------------------------------
// rpcPagedAllocator
//--------------------------------------------------------------------------------------------------
rpcPagedAllocator::rpcPagedAllocator( i32 elementSize, i32 elementsPerPage )
{
	m_blockSize = elementSize;
	m_blocksPerPage = elementsPerPage;

	m_pages = NULL;
	m_pageCount = 0;

	m_freeList = NULL;
}

//--------------------------------------------------------------------------------------------------
rpcPagedAllocator::~rpcPagedAllocator( )
{
	Clear( );
}

//--------------------------------------------------------------------------------------------------
void* rpcPagedAllocator::Allocate( )
{
	if ( m_freeList )
	{
		rpcBlock* data = m_freeList;
		m_freeList = data->next;

		return data;
	}

	else
	{
		rpcPage* page = (rpcPage*)rpcAlloc( m_blockSize * m_blocksPerPage + sizeof( rpcPage ) );
		++m_pageCount;

		page->next = m_pages;
		page->data = (rpcBlock*)RPC_PTR_ADD( page, sizeof( rpcPage ) );
		m_pages = page;

		i32 blocksPerPageMinusOne = m_blocksPerPage - 1;
		for ( i32 i = 0; i < blocksPerPageMinusOne; ++i )
		{
			rpcBlock *node = RPC_PTR_ADD( page->data, m_blockSize * i );
			rpcBlock *next = RPC_PTR_ADD( page->data, m_blockSize * (i + 1) );
			node->next = next;
		}

		rpcBlock *last = RPC_PTR_ADD( page->data, m_blockSize * (blocksPerPageMinusOne) );
		last->next = NULL;

		m_freeList = page->data->next;

		return page->data;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcPagedAllocator::Free( void* data )
{
#ifdef _DEBUG
	bool found = false;

	for ( rpcPage *page = m_pages; page; page = page->next )
	{
		if ( data >= page->data && data < RPC_PTR_ADD( page->data, m_blockSize * m_blocksPerPage ) )
		{
			found = true;
			break;
		}
	}

	// Address of data does not lie within any pages of this allocator.
	assert( found );
#endif // DEBUG

	((rpcBlock*)data)->next = m_freeList;
	m_freeList = ((rpcBlock*)data);
}

//--------------------------------------------------------------------------------------------------
void rpcPagedAllocator::Clear( )
{
	rpcPage* page = m_pages;

	for ( i32 i = 0; i < m_pageCount; ++i )
	{
		rpcPage* next = page->next;
		rpcFree( page );
		page = next;
	}

	m_freeList = NULL;
	m_pageCount = 0;
}
