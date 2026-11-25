//--------------------------------------------------------------------------------------------------
/**
@file	rpcWorkerPool.h

	Small persistent worker pool for parallel island solving (D9, see
	rpc-refactor.md). Tasks are index ranges executed by every thread (the
	caller included) against a shared atomic counter — no allocation, no
	mutex, pure spin synchronization, so a 240 Hz physics loop pays only a
	few microseconds per batch. Numerics are IDENTICAL to serial execution:
	islands share no writable state, only the execution order changes.
*/
//--------------------------------------------------------------------------------------------------

#ifndef RPCWORKERPOOL_H
#define RPCWORKERPOOL_H

#include <atomic>
#include <thread>
#include <vector>

#include "../math/rpcMath.h"

//--------------------------------------------------------------------------------------------------
// rpcWorkerPool
//--------------------------------------------------------------------------------------------------
class rpcWorkerPool
{
public:
	// workerCount: dedicated workers (the caller participates too, so the
	// total parallelism is workerCount + 1). 0 disables threading.
	explicit rpcWorkerPool( i32 workerCount );
	~rpcWorkerPool( );

	// Run fn(0..taskCount-1, arg) across the pool; blocks until every task
	// finished. fn must be thread-safe for disjoint indices.
	void Run( void ( *fn )( i32, void* ), void* arg, i32 taskCount );

	// Dedicated workers (the caller participates too).
	i32 WorkerCount( ) const { return (i32)m_workers.size( ); }

	// Process-wide shared pool: created once at first use (game startup),
	// sized to the machine's CPU count, reused by every scene and never
	// resized or destroyed mid-run — no lazy thread creation on the physics
	// hot path, no per-scene pools.
	static rpcWorkerPool& Shared( );

private:
	void WorkerMain( );

	std::vector<std::thread> m_workers;
	std::atomic<i32> m_quit{ 0 };
	// Hot counters get their own cache line: every worker hammers
	// m_nextTask/m_tasksDone, and sharing one line turns a batch into a
	// coherence ping-pong between the caller and every worker.
	alignas(64) std::atomic<i32> m_seq{ 0 };        // batch generation; ++ on Run()
	alignas(64) std::atomic<i32> m_nextTask{ -1 };  // next unclaimed task index
	alignas(64) std::atomic<i32> m_tasksDone{ 0 };
	alignas(64) std::atomic<i32> m_taskCount{ 0 };
	void ( *m_fn )( i32, void* ) = nullptr;
	void* m_arg = nullptr;
};

#endif // RPCWORKERPOOL_H
