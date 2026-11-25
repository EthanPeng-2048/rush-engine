//--------------------------------------------------------------------------------------------------
/**
@file	rpcDynamicTreeRayIndex.cpp
*/
//--------------------------------------------------------------------------------------------------

#include "rpcDynamicTreeRayIndex.h"
#include "../scene/rpcScene.h"
#include "../collision/rpcBox.h"
#include "../dynamics/rpcBody.h"

namespace {

// Adapts the tree's leaf callback to the scene's shape callback: run the exact
// box test, then report.
struct RayLeafCallback
{
	bool TreeCallBack( i32 id )
	{
		rpcBox* box = static_cast<rpcBox*>( tree.GetUserData( id ) );

		if ( box->body == ignore )
			return true;

		if ( box->Raycast( box->body->GetTransform( ), ray ) )
		{
			return cb->ReportShape( box );
		}

		return true;
	}

	const rpcDynamicAABBTree& tree;
	rpcQueryCallback* cb;
	rpcRaycastData* ray;
	const rpcBody* ignore;
};

}  // namespace

//--------------------------------------------------------------------------------------------------
void rpcDynamicTreeRayIndex::Add( rpcBox* box, const rpcAABB& fatAabb )
{
	box->rayIndexId = m_tree.Insert( fatAabb, box );
}

//--------------------------------------------------------------------------------------------------
void rpcDynamicTreeRayIndex::Remove( rpcBox* box )
{
	if ( box->rayIndexId >= 0 )
	{
		m_tree.Remove( box->rayIndexId );
		box->rayIndexId = -1;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcDynamicTreeRayIndex::Update( rpcBox* box, const rpcAABB& fatAabb )
{
	if ( box->rayIndexId < 0 )
	{
		Add( box, fatAabb );
		return;
	}

	m_tree.Update( box->rayIndexId, fatAabb );
}

//--------------------------------------------------------------------------------------------------
void rpcDynamicTreeRayIndex::Clear( )
{
	// Rebuilding the tree from scratch is the caller's job (shapes are
	// re-registered when the scene is rebuilt).
	while ( m_tree.Count( ) > 0 )
		m_tree.Remove( 0 );
}

//--------------------------------------------------------------------------------------------------
void rpcDynamicTreeRayIndex::Raycast( rpcQueryCallback* cb, rpcRaycastData& ray, const rpcBody* ignore ) const
{
	RayLeafCallback wrapper{ m_tree, cb, &ray, ignore };
	m_tree.Query( &wrapper, ray );
}
