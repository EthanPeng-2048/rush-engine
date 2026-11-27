//--------------------------------------------------------------------------------------------------
/**
@file	rpcWorkerPool.cpp
*/
//--------------------------------------------------------------------------------------------------

#include "rpcWorkerPool.h"

#include <chrono>

// Spin-hint: SSE pause on x86, no-op elsewhere (WASM / ARM).
#if defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
#include <immintrin.h>
#define RPC_CPU_PAUSE() _mm_pause()
#else
#define RPC_CPU_PAUSE() ((void)0)
#endif

//--------------------------------------------------------------------------------------------------
// rpcWorkerPool
//--------------------------------------------------------------------------------------------------
rpcWorkerPool::rpcWorkerPool( i32 workerCount )
{
	for ( i32 i = 0; i < workerCount; ++i )
		m_workers.emplace_back( [this] { WorkerMain( ); } );
}

rpcWorkerPool::~rpcWorkerPool( )
{
	m_quit.store( 1, std::memory_order_release );
	for ( std::thread& t : m_workers )
		t.join( );
}

// Process-wide shared pool: created at first use (game startup / bench
// enable), sized to the CPU count, owned for the whole process. Every scene
// and backend reuses it — never created on the physics hot path, never
// resized, never torn down between races.
rpcWorkerPool& rpcWorkerPool::Shared( )
{
	static rpcWorkerPool pool( [] {
		const i32 hw = (i32)std::thread::hardware_concurrency( );
		return hw > 1 ? hw - 1 : 0;
	}( ) );
	return pool;
}

void rpcWorkerPool::Run( void ( *fn )( i32, void* ), void* arg, i32 taskCount )
{
	if ( m_workers.empty( ) || taskCount <= 1 )
	{
		for ( i32 i = 0; i < taskCount; ++i )
			fn( i, arg );
		return;
	}

	m_fn = fn;
	m_arg = arg;
	m_taskCount.store( taskCount, std::memory_order_relaxed );
	m_nextTask.store( 0, std::memory_order_relaxed );
	m_tasksDone.store( 0, std::memory_order_relaxed );
	m_seq.fetch_add( 1, std::memory_order_release );  // release the batch

	// The caller participates in the batch; every finished task (caller and
	// workers alike) counts toward m_tasksDone, so the completion wait below
	// cannot miss tasks the caller claimed itself.
	for ( ;; )
	{
		const i32 t = m_nextTask.fetch_add( 1, std::memory_order_relaxed );
		if ( t >= taskCount )
			break;
		fn( t, arg );
		m_tasksDone.fetch_add( 1, std::memory_order_acq_rel );
	}

	// Spin until every task finished. The batch is bounded by the 240 Hz
	// physics loop; a few microseconds of spinning beats a condition-variable
	// round trip or a yield (which can cost tens of microseconds of scheduler
	// latency on Windows).
	const i32 want = taskCount;
	while ( m_tasksDone.load( std::memory_order_acquire ) != want )
	{
		RPC_CPU_PAUSE( );
	}
}

void rpcWorkerPool::WorkerMain( )
{
	// Wait for the first batch.
	i32 mySeq = m_seq.load( std::memory_order_acquire );
	while ( !m_quit.load( std::memory_order_acquire ) )
	{
		// Back-off wait: spin a short while (batch handoff is sub-
		// microsecond between physics steps), then yield so an idle pool
		// does not burn a core for the whole session.
		const i32 seq = m_seq.load( std::memory_order_acquire );
		if ( seq == mySeq )
		{
			for ( i32 i = 0; i < 2000 && !m_quit.load( std::memory_order_acquire ) &&
			              m_seq.load( std::memory_order_acquire ) == mySeq;
			      ++i )
			{
				RPC_CPU_PAUSE( );
			}
			if ( m_seq.load( std::memory_order_acquire ) == mySeq )
				std::this_thread::yield( );
			continue;
		}
		mySeq = seq;

		// Claim tasks until the batch is drained. Every finished task counts
		// toward m_tasksDone (the caller counts its own in Run), so the
		// completion wait sees the full total.
		for ( ;; )
		{
			const i32 t = m_nextTask.fetch_add( 1, std::memory_order_relaxed );
			if ( t >= m_taskCount.load( std::memory_order_relaxed ) )
				break;
			m_fn( t, m_arg );
			m_tasksDone.fetch_add( 1, std::memory_order_acq_rel );
		}
	}
}
