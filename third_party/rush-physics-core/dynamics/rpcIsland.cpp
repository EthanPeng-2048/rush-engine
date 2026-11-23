//--------------------------------------------------------------------------------------------------
/**
@file	rpcIsland.cpp

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

#include "rpcIsland.h"
#include "rpcBody.h"
#include "../common/rpcMemory.h"
#include "rpcContactSolver.h"
#include "../common/rpcSettings.h"
#include "../broadphase/rpcBroadPhase.h"
#include "rpcContact.h"

//--------------------------------------------------------------------------------------------------
// rpcIsland
//--------------------------------------------------------------------------------------------------
void rpcIsland::Reset( std::span<rpcBody*> bodies, std::span<rpcVelocityState> velocities,
                       std::span<rpcContactConstraint*> contacts,
                       std::span<rpcContactConstraintState> contactStates, r32 dt,
                       const rpcVec3& gravity, i32 iterations, bool allowSleep,
                       bool enableFriction )
{
	m_bodies = bodies;
	m_velocities = velocities;
	m_contacts = contacts;
	m_contactStates = contactStates;
	m_bodyCount = 0;
	m_contactCount = 0;
	m_dt = dt;
	m_gravity = gravity;
	m_iterations = iterations;
	m_allowSleep = allowSleep;
	m_enableFriction = enableFriction;
}

void rpcIsland::Solve( )
{
	// Apply gravity
	// Integrate velocities and create state buffers, calculate world inertia
	for ( i32 i = 0; i < m_bodyCount; ++i )
	{
		rpcBody *body = m_bodies[ i ];
		rpcVelocityState *v = m_velocities.data( ) + i;

		if ( body->m_flags & rpcBody::eDynamic )
		{
			body->ApplyLinearForce( m_gravity * body->m_gravityScale );

			// Calculate world space intertia tensor
			rpcMat3 r = body->m_tx.rotation;
			body->m_invInertiaWorld = r * body->m_invInertiaModel * rpcTranspose( r );

			// Integrate velocity
			body->m_linearVelocity += (body->m_force * body->m_invMass) * m_dt;
			body->m_angularVelocity += (body->m_invInertiaWorld * body->m_torque) * m_dt;

			// From Box2D!
			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			body->m_linearVelocity *= r32( 1.0 ) / (r32( 1.0 ) + m_dt * body->m_linearDamping);
			body->m_angularVelocity *= r32( 1.0 ) / (r32( 1.0 ) + m_dt * body->m_angularDamping);
		}

		v->v = body->m_linearVelocity;
		v->w = body->m_angularVelocity;
	}

	// Create contact solver, pass in state buffers, create buffers for contacts
	// Initialize velocity constraint for normal + friction and warm start
	rpcContactSolver contactSolver;
	contactSolver.Initialize( this );
	contactSolver.PreSolve( m_dt );

	// Solve contacts
	for ( i32 i = 0; i < m_iterations; ++i )
		contactSolver.Solve( );

	contactSolver.ShutDown( );

	// Copy back state buffers
	// Integrate positions
	for ( i32 i = 0; i < m_bodyCount; ++i )
	{
		rpcBody *body = m_bodies[ i ];
		rpcVelocityState *v = m_velocities.data( ) + i;

		if ( body->m_flags & rpcBody::eStatic )
			continue;

		body->m_linearVelocity = v->v;
		body->m_angularVelocity = v->w;

		// Integrate position
		body->m_worldCenter += body->m_linearVelocity * m_dt;
		body->m_q.Integrate( body->m_angularVelocity, m_dt );
		body->m_q = rpcNormalize( body->m_q );
		body->m_tx.rotation = body->m_q.ToMat3( );
	}

	if ( m_allowSleep )
	{
		// Find minimum sleep time of the entire island
		f32 minSleepTime = RPC_R32_MAX;
		for ( i32 i = 0; i < m_bodyCount; ++i )
		{
			rpcBody* body = m_bodies[ i ];

			if ( body->m_flags & rpcBody::eStatic )
				continue;

			const r32 sqrLinVel = rpcDot( body->m_linearVelocity, body->m_linearVelocity );
			const r32 cbAngVel = rpcDot( body->m_angularVelocity, body->m_angularVelocity );
			const r32 linTol = RPC_SLEEP_LINEAR;
			const r32 angTol = RPC_SLEEP_ANGULAR;

			if ( sqrLinVel > linTol || cbAngVel > angTol )
			{
				minSleepTime = r32( 0.0 );
				body->m_sleepTime = r32( 0.0 );
			}

			else
			{
				body->m_sleepTime += m_dt;
				minSleepTime = rpcMin( minSleepTime, body->m_sleepTime );
			}
		}

		// Put entire island to sleep so long as the minimum found sleep time
		// is below the threshold. If the minimum sleep time reaches below the
		// sleeping threshold, the entire island will be reformed next step
		// and sleep test will be tried again.
		if ( minSleepTime > RPC_SLEEP_TIME )
		{
			for ( i32 i = 0; i < m_bodyCount; ++i )
				m_bodies[ i ]->SetToSleep( );
		}
	}
}

//--------------------------------------------------------------------------------------------------
void rpcIsland::Add( rpcBody *body )
{
	body->m_islandIndex = m_bodyCount;
	m_bodies[ m_bodyCount++ ] = body;
}

//--------------------------------------------------------------------------------------------------
void rpcIsland::Add( rpcContactConstraint *contact )
{
	m_contacts[ m_contactCount++ ] = contact;
}

//--------------------------------------------------------------------------------------------------
void rpcIsland::Initialize( )
{
	for ( i32 i = 0; i < m_contactCount; ++i )
	{
		rpcContactConstraint *cc = m_contacts[ i ];

		rpcContactConstraintState *c = m_contactStates.data( ) + i;

		c->centerA = cc->bodyA->m_worldCenter;
		c->centerB = cc->bodyB->m_worldCenter;
		c->iA = cc->bodyA->m_invInertiaWorld;
		c->iB = cc->bodyB->m_invInertiaWorld;
		c->mA = cc->bodyA->m_invMass;
		c->mB = cc->bodyB->m_invMass;
		c->restitution = cc->restitution;
		c->friction = cc->friction;
		c->indexA = cc->bodyA->m_islandIndex;
		c->indexB = cc->bodyB->m_islandIndex;
		c->normal = cc->manifold.normal;
		c->tangentVectors[ 0 ] = cc->manifold.tangentVectors[ 0 ];
		c->tangentVectors[ 1 ] = cc->manifold.tangentVectors[ 1 ];
		c->contactCount = cc->manifold.contactCount;

		for ( i32 j = 0; j < c->contactCount; ++j )
		{
			rpcContactState *s = c->contacts + j;
			rpcContact *cp = cc->manifold.contacts + j;
			s->ra = cp->position - c->centerA;
			s->rb = cp->position - c->centerB;
			s->penetration = cp->penetration;
			s->normalImpulse = cp->normalImpulse;
			s->tangentImpulse[ 0 ] = cp->tangentImpulse[ 0 ];
			s->tangentImpulse[ 1 ] = cp->tangentImpulse[ 1 ];
		}
	}
}
