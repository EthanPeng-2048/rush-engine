//--------------------------------------------------------------------------------------------------
// rpcBox.inl
//
//	Copyright (c) 2014 Randy Gaul http://www.randygaul.net
//
//	This software is provided 'as-is', without any express or implied
//	warranty. In no event will the authors be held liable for any damages
//	arising from the use of this software.
//
//	Permission is granted to anyone to use this software for any purpose,
//	including commercial applications, and to alter it and redistribute it
//	freely, subject to the following restrictions:
//	  1. The origin of this software must not be misrepresented; you must not
//	     claim that you wrote the original software. If you use this software
//	     in a product, an acknowledgment in the product documentation would be
//	     appreciated but is not required.
//	  2. Altered source versions must be plainly marked as such, and must not
//	     be misrepresented as being the original software.
//	  3. This notice may not be removed or altered from any source distribution.
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// rpcBox
//--------------------------------------------------------------------------------------------------
inline void rpcBox::SetUserdata( void* data ) const
{
	userData = data;
}

//--------------------------------------------------------------------------------------------------
inline void* rpcBox::GetUserdata( ) const
{
	return userData;
}

//--------------------------------------------------------------------------------------------------
// rpcBoxDef
//--------------------------------------------------------------------------------------------------
inline void rpcBoxDef::Set( const rpcTransform& tx, const rpcVec3& extents )
{
	m_tx = tx;
	m_e = extents * r32( 0.5 );
}

//--------------------------------------------------------------------------------------------------
inline void rpcBoxDef::SetRestitution( r32 restitution )
{
	m_restitution = restitution;
}

//--------------------------------------------------------------------------------------------------
inline void rpcBoxDef::SetFriction( r32 friction )
{
	m_friction = friction;
}

//--------------------------------------------------------------------------------------------------
inline void rpcBoxDef::SetDensity( r32 density )
{
	m_density = density;
}

//--------------------------------------------------------------------------------------------------
inline void rpcBoxDef::SetSensor( bool sensor )
{
	m_sensor = sensor;
}

//--------------------------------------------------------------------------------------------------
inline bool rpcBox::HasLocalRotation( ) const
{
	const r32 eps = r32( 1.0e-6 );
	const rpcMat3& r = local.rotation;
	return rpcAbs( r.ex.x - r32( 1.0 ) ) > eps || rpcAbs( r.ey.y - r32( 1.0 ) ) > eps ||
	       rpcAbs( r.ez.z - r32( 1.0 ) ) > eps || rpcAbs( r.ex.y ) > eps || rpcAbs( r.ex.z ) > eps ||
	       rpcAbs( r.ey.x ) > eps || rpcAbs( r.ey.z ) > eps || rpcAbs( r.ez.x ) > eps ||
	       rpcAbs( r.ez.y ) > eps;
}
