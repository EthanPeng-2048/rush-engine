//--------------------------------------------------------------------------------------------------
/**
@file	rpcRayIndex.h

	Pluggable ray-query accelerator for collision shapes.

	The scene owns one `rpcRayIndex` per acceleration domain (static geometry,
	dynamic bodies) and reaches them only through this interface, so the query
	strategy can be swapped at construction time without touching the contact
	pipeline, the solver or gameplay code.

	Contract (identical to the broadphase tree's ray query, so results are
	interchangeable):
	  - `Raycast` reports every box whose *fat* AABB meets the segment, in any
	    order, by calling `cb->ReportShape( box )` after running the exact
	    box/ray test itself. The callback owns the closest-hit bookkeeping and
	    may prune by shrinking `ray.t`; returning false ends the query.
	  - Implementations must never reject a box the exact test would hit
	    (conservative pruning only), otherwise the closest-hit result would
	    differ between strategies.
*/
//--------------------------------------------------------------------------------------------------

#ifndef RPCRAYINDEX_H
#define RPCRAYINDEX_H

#include "../common/rpcTypes.h"

struct rpcBox;
struct rpcAABB;
struct rpcRaycastData;
class rpcQueryCallback;
class rpcBody;

//--------------------------------------------------------------------------------------------------
// rpcRayIndex
//--------------------------------------------------------------------------------------------------
class rpcRayIndex
{
public:
	virtual ~rpcRayIndex( ) { }

	// Human-readable strategy name (bench output / diagnostics).
	virtual const char* Name( ) const = 0;
	// Optional capability: spatial cell count, 0 for accelerators without cells.
	virtual i32 CellCount( ) const { return 0; }

	// Insert a shape with its already-fattened world AABB.
	virtual void Add( rpcBox* box, const rpcAABB& fatAabb ) = 0;
	virtual void Remove( rpcBox* box ) = 0;
	// Called when a shape moved past its fat AABB (dyadic update).
	virtual void Update( rpcBox* box, const rpcAABB& fatAabb ) = 0;
	virtual void Clear( ) = 0;

	// Report candidate shapes along `ray` (see the contract above).
	// `ignore` (optional) lets an implementation skip a whole body before the
	// exact test — a vehicle querying its own four suspension rays would
	// otherwise pay for its own shapes on every ray.
	virtual void Raycast( rpcQueryCallback* cb, rpcRaycastData& ray, const rpcBody* ignore = nullptr ) const = 0;

	// Batch form: same result as `count` separate Raycast calls. Default loops
	// the single-ray form (out of line — see rpcRayIndex.cpp); implementations
	// whose cost is per-visit rather than per-ray override it to amortise the
	// walk across the batch.
	virtual void RaycastBatch( rpcQueryCallback* cb, rpcRaycastData* rays, i32 count,
	                          const rpcBody* ignore = nullptr ) const;
};


//--------------------------------------------------------------------------------------------------
// rpcDynamicBodySource
//--------------------------------------------------------------------------------------------------
// Bodies the dynamic sweep may report. Inverted dependency: the sweep does not
// know the scene, the scene exposes itself through this interface (and tests
// can supply their own list).
class rpcDynamicBodySource
{
public:
	virtual ~rpcDynamicBodySource( ) { }
	virtual rpcBody** Bodies( ) = 0;
	virtual i32 BodyCount( ) const = 0;
};

//--------------------------------------------------------------------------------------------------
// rpcRayIndexFactory
//--------------------------------------------------------------------------------------------------
// Construction-time dependency injection: the scene asks the factory for its
// accelerators instead of hard-coding a strategy. Production uses the built-in
// pair (static grid + dynamic sweep); tests inject mocks, and an alternate
// accelerator only has to implement rpcRayIndex.
class rpcRayIndexFactory
{
public:
	virtual ~rpcRayIndexFactory( ) { }
	// Static-geometry accelerator (may return NULL to disable that domain).
	virtual rpcRayIndex* CreateStaticIndex( r32 cellSize ) = 0;
	// Dynamic-body accelerator (may return NULL).
	virtual rpcRayIndex* CreateDynamicIndex( rpcDynamicBodySource* source ) = 0;
};

#endif // RPCRAYINDEX_H
