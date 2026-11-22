//--------------------------------------------------------------------------------------------------
/**
@file	rpcMemory.h

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

#ifndef RPCMEMORY_H
#define RPCMEMORY_H

#include <stdlib.h>

#include "rpcTypes.h"

//--------------------------------------------------------------------------------------------------
// Memory Macros
//--------------------------------------------------------------------------------------------------
inline void* rpcAlloc( i32 bytes )
{
	return malloc( bytes );
}

inline void rpcFree( void* memory )
{
	free( memory );
}

#define RPC_PTR_ADD( P, BYTES ) \
	((decltype( P ))(((u8 *)P) + (BYTES)))

//--------------------------------------------------------------------------------------------------
// rpcStack
//--------------------------------------------------------------------------------------------------

class rpcStack
{
private:
	struct rpcStackEntry
	{
		u8 *data;
		i32 size;
	};

public:
	rpcStack( );
	~rpcStack( );

	void Reserve( u32 size );
	void *Allocate( i32 size );
	void Free( void *data );

private:
	u8* m_memory;
	rpcStackEntry* m_entries;

	u32 m_index;

	i32 m_allocation;
	i32 m_entryCount;
	i32 m_entryCapacity;
	u32 m_stackSize;
};

//--------------------------------------------------------------------------------------------------
// rpcHeap
//--------------------------------------------------------------------------------------------------
// 20 MB heap size, change as necessary
const i32 rpck_heapSize = 1024 * 1024 * 20;
const i32 rpck_heapInitialCapacity = 1024;

// Operates on first fit basis in attempt to improve cache coherency
class rpcHeap
{
private:
	struct rpcHeader
	{
		rpcHeader* next;
		rpcHeader* prev;
		i32 size;
	};

	struct rpcFreeBlock
	{
		rpcHeader* header;
		i32 size;
	};

public:
	rpcHeap( );
	~rpcHeap( );

	void *Allocate( i32 size );
	void Free( void *memory );

private:
	rpcHeader* m_memory;

	rpcFreeBlock* m_freeBlocks;
	i32 m_freeBlockCount;
	i32 m_freeBlockCapacity;
};

//--------------------------------------------------------------------------------------------------
// rpcPagedAllocator
//--------------------------------------------------------------------------------------------------
class rpcPagedAllocator
{
	struct rpcBlock
	{
		rpcBlock* next;
	};

	struct rpcPage
	{
		rpcPage* next;
		rpcBlock* data;
	};

public:
	rpcPagedAllocator( i32 elementSize, i32 elementsPerPage );
	~rpcPagedAllocator( );

	void* Allocate( );
	void Free( void* data );

	void Clear( );

private:
	i32 m_blockSize;
	i32 m_blocksPerPage;

	rpcPage *m_pages;
	i32 m_pageCount;

	rpcBlock *m_freeList;
};

#endif // RPCMEMORY_H
