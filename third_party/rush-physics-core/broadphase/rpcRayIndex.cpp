//--------------------------------------------------------------------------------------------------
/**
@file	rpcRayIndex.cpp

	Default batch implementation for ray indexes that only implement the
	single-ray contract.
*/
//--------------------------------------------------------------------------------------------------

#include "rpcRayIndex.h"
#include "../scene/rpcScene.h"
#include "../dynamics/rpcBody.h"

namespace {

// Binds one ray index for a batch callback (the scene's contract is
// ReportShape( box, rayIndex, ray )).
class BatchForward final : public rpcQueryCallback
{
public:
	bool ReportShape( rpcBox* box ) override
	{
		return target->ReportShape( box, index, &rays[ index ] );
	}

	rpcQueryCallback* target;
	i32 index;
	rpcRaycastData* rays;
};

}  // namespace

//--------------------------------------------------------------------------------------------------
void rpcRayIndex::RaycastBatch( rpcQueryCallback* cb, rpcRaycastData* rays, i32 count,
                                 const rpcBody* ignore ) const
{
	for ( i32 j = 0; j < count; ++j )
	{
		BatchForward forward;
		forward.target = cb;
		forward.index = j;
		forward.rays = rays;
		Raycast( &forward, rays[ j ], ignore );
	}
}
