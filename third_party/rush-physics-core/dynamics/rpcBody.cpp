//--------------------------------------------------------------------------------------------------
/**
@file	rpcBody.cpp

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

#include "rpcBody.h"
#include "../scene/rpcScene.h"
#include "rpcContact.h"
#include "../broadphase/rpcBroadPhase.h"
#include "../collision/rpcBox.h"

//--------------------------------------------------------------------------------------------------
// rpcBody
//--------------------------------------------------------------------------------------------------
rpcBody::rpcBody( const rpcBodyDef& def, rpcScene* scene )
{
	m_linearVelocity = def.linearVelocity;
	m_angularVelocity = def.angularVelocity;
	rpcIdentity( m_force );
	rpcIdentity( m_torque );
	m_q.Set( rpcNormalize( def.axis ), def.angle );
	m_tx.rotation = m_q.ToMat3( );
	m_tx.position = def.position;
	m_sleepTime = r32( 0.0 );
	m_gravityScale = def.gravityScale;
	m_layers = def.layers;
	m_userData = def.userData;
	m_scene = scene;
	m_uid = scene->NextBodyUid( );
	m_flags = 0;
	m_linearDamping = def.linearDamping;
	m_angularDamping = def.angularDamping;

	if ( def.bodyType == eDynamicBody )
		m_flags |= rpcBody::eDynamic;

	else
	{
		if ( def.bodyType == eStaticBody )
		{
			m_flags |= rpcBody::eStatic;
			rpcIdentity( m_linearVelocity );
			rpcIdentity( m_angularVelocity );
			rpcIdentity( m_force );
			rpcIdentity( m_torque );
		}

		else if ( def.bodyType == eKinematicBody )
			m_flags |= rpcBody::eKinematic;
	}

	if ( def.allowSleep )
		m_flags |= eAllowSleep;

	if ( def.awake )
		m_flags |= eAwake;

	if ( def.active )
		m_flags |= eActive;

	if ( def.lockAxisX )
		m_flags |= eLockAxisX;

	if ( def.lockAxisY )
		m_flags |= eLockAxisY;

	if ( def.lockAxisZ )
		m_flags |= eLockAxisZ;

	m_boxes = NULL;
	m_contactList = NULL;

	// Empty accumulator: rpcIsEmpty() drives the first-union special case.
	m_fatAabb.min = rpcVec3( RPC_R32_MAX, RPC_R32_MAX, RPC_R32_MAX );
	m_fatAabb.max = rpcVec3( -RPC_R32_MAX, -RPC_R32_MAX, -RPC_R32_MAX );
}

//--------------------------------------------------------------------------------------------------
const rpcBox* rpcBody::AddBox( const rpcBoxDef& def )
{
	rpcAABB aabb;
	rpcBox* box = (rpcBox*)m_scene->m_heap.Allocate( sizeof( rpcBox ) );
	box->local = def.m_tx;
	box->e = def.m_e;
	box->next = m_boxes;
	m_boxes = box;
	box->ComputeAABB( m_tx, &aabb );

	box->body = this;
	box->friction = def.m_friction;
	box->restitution = def.m_restitution;
	box->density = def.m_density;
	box->sensor = def.m_sensor;

	box->uid = m_scene->NextBoxUid( );
	box->rayIndexId = -1;
	box->fatAABB = rpcFatten( aabb );

	CalculateMassData( );

	m_scene->m_contactManager.m_broadphase.InsertBox( box, aabb );
	m_scene->RegisterBoxProxy( box, box->fatAABB );
	m_scene->m_newBox = true;

	// Body proxy = union of its shapes (the dynamic ray sweep scans these).
	if ( rpcIsEmpty( m_fatAabb ) )
		m_fatAabb = box->fatAABB;
	else
		rpcUnion( m_fatAabb, box->fatAABB );

	return box;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::RemoveBox( const rpcBox* box )
{
	assert( box );
	assert( box->body == this );

	rpcBox* node = m_boxes;

	bool found = false;
	if ( node == box )
	{
		m_boxes = node->next;
		found = true;
	}

	else
	{
		while ( node )
		{
			if ( node->next == box )
			{
				node->next = box->next;
				found = true;
				break;
			}

			node = node->next;
		}
	}

	// This shape was not connected to this body.
	assert( found );

	// Remove all contacts associated with this shape
	rpcContactEdge* edge = m_contactList;
	while ( edge )
	{
		rpcContactConstraint* contact = edge->constraint;
		edge = edge->next;

		rpcBox* A = contact->A;
		rpcBox* B = contact->B;

		if ( box == A || box == B )
			m_scene->m_contactManager.RemoveContact( contact );
	}

	m_scene->m_contactManager.m_broadphase.RemoveBox( box );
	m_scene->UnregisterBoxProxy( (rpcBox*)box );

	CalculateMassData( );
	RefreshFatAabb( );

	m_scene->m_heap.Free( (void*)box );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::RemoveAllBoxes( )
{
	while ( m_boxes )
	{
		rpcBox* next = m_boxes->next;

		m_scene->m_contactManager.m_broadphase.RemoveBox( m_boxes );
		m_scene->UnregisterBoxProxy( m_boxes );
		m_scene->m_heap.Free( (void*)m_boxes );

		m_boxes = next;
	}

	m_scene->m_contactManager.RemoveContactsFromBody( this );

	m_fatAabb.min = rpcVec3( RPC_R32_MAX, RPC_R32_MAX, RPC_R32_MAX );
	m_fatAabb.max = rpcVec3( -RPC_R32_MAX, -RPC_R32_MAX, -RPC_R32_MAX );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::RefreshFatAabb( )
{
	bool first = true;
	rpcAABB out;

	for ( rpcBox* box = m_boxes; box; box = box->next )
	{
		if ( first )
		{
			out = box->fatAABB;
			first = false;
		}
		else
			rpcUnion( out, box->fatAABB );
	}

	if ( first )
	{
		out.min = rpcVec3( RPC_R32_MAX, RPC_R32_MAX, RPC_R32_MAX );
		out.max = rpcVec3( -RPC_R32_MAX, -RPC_R32_MAX, -RPC_R32_MAX );
	}

	m_fatAabb = out;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::ApplyLinearForce( const rpcVec3& force )
{
	m_force += force * m_mass;

	SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::ApplyForceAtWorldPoint( const rpcVec3& force, const rpcVec3& point )
{
	m_force += force * m_mass;
	m_torque += rpcCross( point - m_worldCenter, force );

	SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::ApplyLinearImpulse( const rpcVec3& impulse )
{
    m_linearVelocity += impulse * m_invMass;

    SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::ApplyLinearImpulseAtWorldPoint( const rpcVec3& impulse, const rpcVec3& point )
{
    m_linearVelocity += impulse * m_invMass;
    m_angularVelocity += m_invInertiaWorld * rpcCross( point - m_worldCenter, impulse );

    SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::ApplyAngularImpulse( const rpcVec3& worldImpulse )
{
    m_angularVelocity += m_invInertiaWorld * worldImpulse;

    SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::ApplyTorque( const rpcVec3& torque )
{
	m_torque += torque;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetToAwake( )
{
	if( !(m_flags & eAwake) )
	{
		m_flags |= eAwake;
		m_sleepTime = r32( 0.0 );
	}
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetToSleep( )
{
	m_flags &= ~eAwake;
	m_sleepTime = r32( 0.0 );
	rpcIdentity( m_linearVelocity );
	rpcIdentity( m_angularVelocity );
	rpcIdentity( m_force );
	rpcIdentity( m_torque );
}

//--------------------------------------------------------------------------------------------------
bool rpcBody::IsAwake( ) const
{
	return m_flags & eAwake ? true : false;
}

//--------------------------------------------------------------------------------------------------
r32 rpcBody::GetMass( ) const
{
    return m_mass;
}

//--------------------------------------------------------------------------------------------------
r32 rpcBody::GetInvMass( ) const
{
    return m_invMass;
}

//--------------------------------------------------------------------------------------------------
r32 rpcBody::GetGravityScale( ) const
{
	return m_gravityScale;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetGravityScale( r32 scale )
{
	m_gravityScale = scale;
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcBody::GetLocalPoint( const rpcVec3& p ) const
{
	return rpcMulT( m_tx, p );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcBody::GetLocalVector( const rpcVec3& v ) const
{
	return rpcMulT( m_tx.rotation, v );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcBody::GetWorldPoint( const rpcVec3& p ) const
{
	return rpcMul( m_tx, p );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcBody::GetWorldVector( const rpcVec3& v ) const
{
	return rpcMul( m_tx.rotation, v );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcBody::GetLinearVelocity( ) const
{
	return m_linearVelocity;
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcBody::GetVelocityAtWorldPoint( const rpcVec3& p ) const
{
	rpcVec3 directionToPoint = p - m_worldCenter;
	rpcVec3 relativeAngularVel = rpcCross( m_angularVelocity, directionToPoint );

	return m_linearVelocity + relativeAngularVel;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetLinearVelocity( const rpcVec3& v )
{
	// Velocity of static bodies cannot be adjusted
	if ( m_flags & eStatic )
		assert( false );

	if ( rpcDot( v, v ) > r32( 0.0 ) )
	{
		SetToAwake( );
	}

	m_linearVelocity = v;
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcBody::GetAngularVelocity( ) const
{
	return m_angularVelocity;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetAngularVelocity( const rpcVec3 v )
{
	// Velocity of static bodies cannot be adjusted
	if ( m_flags & eStatic )
		assert( false );

	if ( rpcDot( v, v ) > r32( 0.0 ) )
	{
		SetToAwake( );
	}

	m_angularVelocity = v;
}

//--------------------------------------------------------------------------------------------------
bool rpcBody::CanCollide( const rpcBody *other ) const
{
	if ( this == other )
		return false;

	// Every collision must have at least one dynamic body involved
	if ( !(m_flags & eDynamic) && !(other->m_flags & eDynamic) )
		return false;

	if ( !(m_layers & other->m_layers) )
		return false;

	return true;
}

//--------------------------------------------------------------------------------------------------
const rpcTransform rpcBody::GetTransform( ) const
{
	return m_tx;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetTransform( const rpcVec3& position )
{
	m_worldCenter = position;

	SynchronizeProxies( );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetTransform( const rpcVec3& position, const rpcVec3& axis, r32 angle )
{
	m_worldCenter = position;
	m_q.Set( axis, angle );
	m_tx.rotation = m_q.ToMat3( );

	SynchronizeProxies( );
}

//--------------------------------------------------------------------------------------------------
i32 rpcBody::GetFlags( ) const
{
	return m_flags;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetLayers( i32 layers )
{
	m_layers = layers;
}

//--------------------------------------------------------------------------------------------------
i32 rpcBody::GetLayers( ) const
{
	return m_layers;
}

//--------------------------------------------------------------------------------------------------
const rpcQuaternion rpcBody::GetQuaternion( ) const
{
	return m_q;
}

//--------------------------------------------------------------------------------------------------
void* rpcBody::GetUserData( ) const
{
  return m_userData;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetLinearDamping( r32 damping )
{
	m_linearDamping = damping;
}

//--------------------------------------------------------------------------------------------------
r32 rpcBody::GetLinearDamping( r32 damping ) const
{
	return m_linearDamping;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SetAngularDamping( r32 damping )
{
	m_angularDamping = damping;
}

//--------------------------------------------------------------------------------------------------
r32 rpcBody::GetAngularDamping( r32 damping ) const
{
	return m_angularDamping;
}

//--------------------------------------------------------------------------------------------------
void rpcBody::Render( rpcRender* render ) const
{
	bool awake = IsAwake( );
	rpcBox* box = m_boxes;

	while ( box )
	{
		box->Render( m_tx, awake, render );
		box = box->next;
	}
}

//--------------------------------------------------------------------------------------------------
void rpcBody::Dump( FILE* file, i32 index ) const
{
	fprintf( file, "{\n" );
	fprintf( file, "\tq3BodyDef bd;\n" );

	switch ( m_flags & (eStatic | eDynamic | eKinematic) )
	{
	case eStatic:
		fprintf( file, "\tbd.bodyType = rpcBodyType( %d );\n", eStaticBody );
		break;

	case eDynamic:
		fprintf( file, "\tbd.bodyType = rpcBodyType( %d );\n", eDynamicBody );
		break;

	case eKinematic:
		fprintf( file, "\tbd.bodyType = rpcBodyType( %d );\n", eKinematicBody );
		break;
	}

	fprintf( file, "\tbd.position.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", m_tx.position.x, m_tx.position.y, m_tx.position.z );
	rpcVec3 axis;
	r32 angle;
	m_q.ToAxisAngle( &axis, &angle );
	fprintf( file, "\tbd.axis.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", axis.x, axis.y, axis.z );
	fprintf( file, "\tbd.angle = r32( %.15lf );\n", angle );
	fprintf( file, "\tbd.linearVelocity.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", m_linearVelocity.x, m_linearVelocity.y, m_linearVelocity.z );
	fprintf( file, "\tbd.angularVelocity.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z );
	fprintf( file, "\tbd.gravityScale = r32( %.15lf );\n", m_gravityScale );
	fprintf( file, "\tbd.layers = %d;\n", m_layers );
	fprintf( file, "\tbd.allowSleep = bool( %d );\n", m_flags & eAllowSleep );
	fprintf( file, "\tbd.awake = bool( %d );\n", m_flags & eAwake );
	fprintf( file, "\tbd.awake = bool( %d );\n", m_flags & eAwake );
	fprintf( file, "\tbd.lockAxisX = bool( %d );\n", m_flags & eLockAxisX );
	fprintf( file, "\tbd.lockAxisY = bool( %d );\n", m_flags & eLockAxisY );
	fprintf( file, "\tbd.lockAxisZ = bool( %d );\n", m_flags & eLockAxisZ );
	fprintf( file, "\tbodies[ %d ] = scene.CreateBody( bd );\n\n", index );

	rpcBox* box = m_boxes;

	while ( box )
	{
		fprintf( file, "\t{\n" );
		fprintf( file, "\t\tq3BoxDef sd;\n" );
		fprintf( file, "\t\tsd.SetFriction( r32( %.15lf ) );\n", box->friction );
		fprintf( file, "\t\tsd.SetRestitution( r32( %.15lf ) );\n", box->restitution );
		fprintf( file, "\t\tsd.SetDensity( r32( %.15lf ) );\n", box->density );
		i32 sensor = (int)box->sensor;
		fprintf( file, "\t\tsd.SetSensor( bool( %d ) );\n", sensor );
		fprintf( file, "\t\tq3Transform boxTx;\n" );
		rpcTransform boxTx = box->local;
		rpcVec3 xAxis = boxTx.rotation.ex;
		rpcVec3 yAxis = boxTx.rotation.ey;
		rpcVec3 zAxis = boxTx.rotation.ez;
		fprintf( file, "\t\tq3Vec3 xAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", xAxis.x, xAxis.y, xAxis.z );
		fprintf( file, "\t\tq3Vec3 yAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", yAxis.x, yAxis.y, yAxis.z );
		fprintf( file, "\t\tq3Vec3 zAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", zAxis.x, zAxis.y, zAxis.z );
		fprintf( file, "\t\tboxTx.rotation.SetRows( xAxis, yAxis, zAxis );\n" );
		fprintf( file, "\t\tboxTx.position.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", boxTx.position.x, boxTx.position.y, boxTx.position.z );
		fprintf( file, "\t\tsd.Set( boxTx, rpcVec3( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) ) );\n", box->e.x * 2.0f, box->e.y * 2.0f, box->e.z * 2.0f );
		fprintf( file, "\t\tbodies[ %d ]->AddBox( sd );\n", index );
		fprintf( file, "\t}\n" );
		box = box->next;
	}

	fprintf( file, "}\n\n" );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::CalculateMassData( )
{
	rpcMat3 inertia = rpcDiagonal( r32( 0.0 ) );
	m_invInertiaModel = rpcDiagonal( r32( 0.0 ) );
	m_invInertiaWorld = rpcDiagonal( r32( 0.0 ) );
	m_invMass = r32( 0.0 );
	m_mass = r32( 0.0 );
	r32 mass = r32( 0.0 );

	if ( m_flags & eStatic || m_flags &eKinematic )
	{
		rpcIdentity( m_localCenter );
		m_worldCenter = m_tx.position;
		return;
	}

	rpcVec3 lc;
	rpcIdentity( lc );

	for ( rpcBox* box = m_boxes; box; box = box->next)
	{
		if ( box->density == r32( 0.0 ) )
			continue;

		rpcMassData md;
		box->ComputeMass( &md );
		mass += md.mass;
		inertia += md.inertia;
		lc += md.center * md.mass;
	}

	if ( mass > r32( 0.0 ) )
	{
		m_mass = mass;
		m_invMass = r32( 1.0 ) / mass;
		lc *= m_invMass;
		rpcMat3 identity;
		rpcIdentity( identity );
		inertia -= (identity * rpcDot( lc, lc ) - rpcOuterProduct( lc, lc )) * mass;
		m_invInertiaModel = rpcInverse( inertia );

		if ( m_flags & eLockAxisX )
			rpcIdentity( m_invInertiaModel.ex );

		if ( m_flags & eLockAxisY )
			rpcIdentity( m_invInertiaModel.ey );

		if ( m_flags & eLockAxisZ )
			rpcIdentity( m_invInertiaModel.ez );
	}
	else
	{
		// Force all dynamic bodies to have some mass
		m_invMass = r32( 1.0 );
		m_invInertiaModel = rpcDiagonal( r32( 0.0 ) );
		m_invInertiaWorld = rpcDiagonal( r32( 0.0 ) );
	}

	m_localCenter = lc;
	m_worldCenter = rpcMul( m_tx, lc );
}

//--------------------------------------------------------------------------------------------------
void rpcBody::SynchronizeProxies( )
{
	rpcBroadPhase* broadphase = &m_scene->m_contactManager.m_broadphase;

	m_tx.position = m_worldCenter - rpcMul( m_tx.rotation, m_localCenter );

	rpcAABB aabb;
	rpcTransform tx = m_tx;
	bool first = true;
	rpcAABB bodyAabb;

	rpcBox* box = m_boxes;
	while ( box )
	{
		box->ComputeAABB( tx, &aabb );
		box->fatAABB = rpcFatten( aabb );

		// One containment test drives both trees: they share the same fat AABB,
		// so "the broadphase re-inserted it" is exactly "the ray index needs
		// to". Skipping the second walk when the shape stayed inside its margin
		// is most of the per-substep proxy cost at 480 Hz.
		if ( broadphase->Update( box->broadPhaseIndex, aabb ) )
			m_scene->UpdateBoxProxy( box, box->fatAABB );

		if ( first )
		{
			bodyAabb = box->fatAABB;
			first = false;
		}
		else
			rpcUnion( bodyAabb, box->fatAABB );

		box = box->next;
	}

	if ( !first )
		m_fatAabb = bodyAabb;
}
