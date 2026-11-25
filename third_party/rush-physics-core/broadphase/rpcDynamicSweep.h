//--------------------------------------------------------------------------------------------------
/**
@file	rpcDynamicSweep.h

	Ray index over the *dynamic* bodies (implementation of rpcRayIndex).

	Why a linear sweep instead of a tree: a race has O(100) dynamic bodies, and
	their proxy AABBs (32 B each) fit in a few kilobytes — well inside L1. A
	single pass over them beats a tree walk because it is sequential, branch
	light and has no pointer chasing. Only bodies whose bounds meet the segment
	are opened, and only their (few) shapes are tested exactly.

	Population comes from `rpcDynamicBodySource` (dependency inversion): the
	scene supplies its body list, tests supply their own.
*/
//--------------------------------------------------------------------------------------------------

#ifndef RPCDYNAMICSWEEP_H
#define RPCDYNAMICSWEEP_H

#include "rpcRayIndex.h"

//--------------------------------------------------------------------------------------------------
// rpcDynamicSweep
//--------------------------------------------------------------------------------------------------
class rpcDynamicSweep final : public rpcRayIndex
{
public:
	explicit rpcDynamicSweep( rpcDynamicBodySource* source ) : m_source( source ) { }

	// rpcRayIndex. The population is enumerated through the source, so the
	// insertion hooks are intentionally no-ops (kept for interface symmetry).
	const char* Name( ) const override { return "dynamic-sweep"; }
	void Add( rpcBox*, const rpcAABB& ) override { }
	void Remove( rpcBox* ) override { }
	void Update( rpcBox*, const rpcAABB& ) override { }
	void Clear( ) override { }
	void Raycast( rpcQueryCallback* cb, rpcRaycastData& ray, const rpcBody* ignore = nullptr ) const override;
	// NOTE: a body-major RaycastBatch override was measured 30% SLOWER than the
	// default per-ray loop (the per-ray rejection early-outs after one axis
	// test, while body-major must evaluate every ray of the batch before it can
	// skip a body). The default implementation is kept on purpose.

	// Bodies whose bounds met the segment during the last query (diagnostics).
	i32 LastCandidates( ) const { return m_lastCandidates; }

private:
	rpcDynamicBodySource* m_source;
	mutable i32 m_lastCandidates = 0;
};

#endif // RPCDYNAMICSWEEP_H
