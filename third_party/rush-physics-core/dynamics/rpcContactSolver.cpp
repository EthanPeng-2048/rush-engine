//--------------------------------------------------------------------------------------------------
/**
@file	rpcContactSolver.cpp

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

#include "rpcContactSolver.h"
#include "rpcContact.h"
#include "rpcIsland.h"
#include "../common/rpcMemory.h"
#include "rpcBody.h"
#include "../common/rpcGeometry.h"
#include "../common/rpcSettings.h"

//--------------------------------------------------------------------------------------------------
// rpcContactSolver
//--------------------------------------------------------------------------------------------------
void rpcContactSolver::Initialize( rpcIsland *island )
{
	m_island = island;
	m_contactCount = island->m_contactCount;
	m_contacts = island->m_contactStates.data( );
	m_velocities = m_island->m_velocities.data( );
	m_enableFriction = island->m_enableFriction;
}

//--------------------------------------------------------------------------------------------------
void rpcContactSolver::ShutDown( void )
{
	for ( i32 i = 0; i < m_contactCount; ++i )
	{
		rpcContactConstraintState *c = m_contacts + i;
		rpcContactConstraint *cc = m_island->m_contacts[ i ];

		for ( i32 j = 0; j < c->contactCount; ++j )
		{
			rpcContact *oc = cc->manifold.contacts + j;
			rpcContactState *cs = c->contacts + j;
			oc->normalImpulse = cs->normalImpulse;
			oc->tangentImpulse[ 0 ] = cs->tangentImpulse[ 0 ];
			oc->tangentImpulse[ 1 ] = cs->tangentImpulse[ 1 ];
		}
	}
}

//--------------------------------------------------------------------------------------------------
void rpcContactSolver::PreSolve( r32 dt )
{
	for ( i32 i = 0; i < m_contactCount; ++i )
	{
		rpcContactConstraintState *cs = m_contacts + i;

		rpcVec3 vA = m_velocities[ cs->indexA ].v;
		rpcVec3 wA = m_velocities[ cs->indexA ].w;
		rpcVec3 vB = m_velocities[ cs->indexB ].v;
		rpcVec3 wB = m_velocities[ cs->indexB ].w;

		for ( i32 j = 0; j < cs->contactCount; ++j )
		{
			rpcContactState *c = cs->contacts + j;

			// Precalculate JM^-1JT for contact and friction constraints
			rpcVec3 raCn = rpcCross( c->ra, cs->normal );
			rpcVec3 rbCn = rpcCross( c->rb, cs->normal );
			r32 nm = cs->mA + cs->mB;
			r32 tm[ 2 ];
			tm[ 0 ] = nm;
			tm[ 1 ] = nm;

			nm += rpcDot( raCn, cs->iA * raCn ) + rpcDot( rbCn, cs->iB * rbCn );
			c->normalMass = rpcInvert( nm );

			for ( i32 i = 0; i < 2; ++i )
			{
				rpcVec3 raCt = rpcCross( cs->tangentVectors[ i ], c->ra );
				rpcVec3 rbCt = rpcCross( cs->tangentVectors[ i ], c->rb );
				tm[ i ] += rpcDot( raCt, cs->iA * raCt ) + rpcDot( rbCt, cs->iB * rbCt );
				c->tangentMass[ i ] = rpcInvert( tm[ i ] );
			}

			// Precalculate bias factor. The penetration depth that feeds the
			// correction velocity is clamped to RPC_MAX_LINEAR_CORRECTION, so
			// deep overlap (pile-up, overlapping spawn) resolves over several
			// steps instead of ejecting the bodies at depth/dt.
			const r32 pen = rpcMax( rpcMin( r32( 0.0 ), c->penetration + RPC_PENETRATION_SLOP ),
			                        -RPC_MAX_LINEAR_CORRECTION );
			c->bias = -RPC_BAUMGARTE * (r32( 1.0 ) / dt) * pen;

			// Warm start contact
			rpcVec3 P = cs->normal * c->normalImpulse;

			if ( m_enableFriction )
			{
				P += cs->tangentVectors[ 0 ] * c->tangentImpulse[ 0 ];
				P += cs->tangentVectors[ 1 ] * c->tangentImpulse[ 1 ];
			}

			vA -= P * cs->mA;
			wA -= cs->iA * rpcCross( c->ra, P );

			vB += P * cs->mB;
			wB += cs->iB * rpcCross( c->rb, P );

			// Add in restitution bias
			r32 dv = rpcDot( vB + rpcCross( wB, c->rb ) - vA - rpcCross( wA, c->ra ), cs->normal );

			if ( dv < -r32( 1.0 ) )
				c->bias += -(cs->restitution) * dv;
		}

		m_velocities[ cs->indexA ].v = vA;
		m_velocities[ cs->indexA ].w = wA;
		m_velocities[ cs->indexB ].v = vB;
		m_velocities[ cs->indexB ].w = wB;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcContactSolver::Solve( )
{
	for ( i32 i = 0; i < m_contactCount; ++i )
	{
		rpcContactConstraintState *cs = m_contacts + i;

		rpcVec3 vA = m_velocities[ cs->indexA ].v;
		rpcVec3 wA = m_velocities[ cs->indexA ].w;
		rpcVec3 vB = m_velocities[ cs->indexB ].v;
		rpcVec3 wB = m_velocities[ cs->indexB ].w;

		for ( i32 j = 0; j < cs->contactCount; ++j )
		{
			rpcContactState *c = cs->contacts + j;

			// relative velocity at contact
			rpcVec3 dv = vB + rpcCross( wB, c->rb ) - vA - rpcCross( wA, c->ra );

			// Friction
			if ( m_enableFriction )
			{
				for ( i32 i = 0; i < 2; ++i )
				{
					r32 lambda = -rpcDot( dv, cs->tangentVectors[ i ] ) * c->tangentMass[ i ];

					// Calculate frictional impulse
					r32 maxLambda = cs->friction * c->normalImpulse;

					// Clamp frictional impulse
					r32 oldPT = c->tangentImpulse[ i ];
					c->tangentImpulse[ i ] = rpcClamp( -maxLambda, maxLambda, oldPT + lambda );
					lambda = c->tangentImpulse[ i ] - oldPT;

					// Apply friction impulse
					rpcVec3 impulse = cs->tangentVectors[ i ] * lambda;
					vA -= impulse * cs->mA;
					wA -= cs->iA * rpcCross( c->ra, impulse );

					vB += impulse * cs->mB;
					wB += cs->iB * rpcCross( c->rb, impulse );
				}
			}

			// Normal
			{
				dv = vB + rpcCross( wB, c->rb ) - vA - rpcCross( wA, c->ra );

				// Normal impulse
				r32 vn = rpcDot( dv, cs->normal );

				// Factor in positional bias to calculate impulse scalar j
				r32 lambda = c->normalMass * (-vn + c->bias);

				// Clamp impulse
				r32 tempPN = c->normalImpulse;
				c->normalImpulse = rpcMax( tempPN + lambda, r32( 0.0 ) );
				lambda = c->normalImpulse - tempPN;

				// Apply impulse
				rpcVec3 impulse = cs->normal * lambda;
				vA -= impulse * cs->mA;
				wA -= cs->iA * rpcCross( c->ra, impulse );

				vB += impulse * cs->mB;
				wB += cs->iB * rpcCross( c->rb, impulse );
			}
		}

		m_velocities[ cs->indexA ].v = vA;
		m_velocities[ cs->indexA ].w = wA;
		m_velocities[ cs->indexB ].v = vB;
		m_velocities[ cs->indexB ].w = wB;
	}
}
