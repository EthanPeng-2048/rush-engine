//--------------------------------------------------------------------------------------------------
// rpcTransform.inl
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
// rpcTransform
//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMul( const rpcTransform& tx, const rpcVec3& v )
{
	return rpcVec3( tx.rotation * v + tx.position );
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMul( const rpcTransform& tx, const rpcVec3& scale, const rpcVec3& v )
{
	return rpcVec3( tx.rotation * rpcMul( scale, v ) + tx.position );
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMul( const rpcTransform *tx, const rpcVec3& v )
{
	return rpcVec3( tx->rotation * v + tx->position );
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMul( const rpcMat3& r, const rpcVec3& v )
{
	return r * v;
}

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcMul( const rpcMat3& r, const rpcMat3& q )
{
	return r * q;
}

//--------------------------------------------------------------------------------------------------
inline const rpcTransform rpcMul( const rpcTransform& t, const rpcTransform& u )
{
	rpcTransform v;
	v.rotation = rpcMul( t.rotation, u.rotation );
	v.position = rpcMul( t.rotation, u.position ) + t.position;
	return v;
}

//--------------------------------------------------------------------------------------------------
inline const rpcHalfSpace rpcMul( const rpcTransform& tx, const rpcHalfSpace& p )
{
	rpcVec3 origin = p.Origin( );
	origin = rpcMul( tx, origin );
	rpcVec3 normal = rpcMul( tx.rotation, p.normal );

	return rpcHalfSpace( normal, rpcDot( origin, normal ) );
}

//--------------------------------------------------------------------------------------------------
inline const rpcHalfSpace rpcMul( const rpcTransform& tx, const rpcVec3& scale, const rpcHalfSpace& p )
{
	rpcVec3 origin = p.Origin( );
	origin = rpcMul( tx, scale, origin );
	rpcVec3 normal = rpcMul( tx.rotation, p.normal );

	return rpcHalfSpace( normal, rpcDot( origin, normal ) );
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMulT( const rpcTransform& tx, const rpcVec3& v )
{
	return rpcTranspose( tx.rotation ) * (v - tx.position);
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcMulT( const rpcMat3& r, const rpcVec3& v )
{
	return rpcTranspose( r ) * v;
}

//--------------------------------------------------------------------------------------------------
inline const rpcMat3 rpcMulT( const rpcMat3& r, const rpcMat3& q )
{
	return rpcTranspose( r ) * q;
}

//--------------------------------------------------------------------------------------------------
inline const rpcTransform rpcMulT( const rpcTransform& t, const rpcTransform& u )
{
	rpcTransform v;
	v.rotation = rpcMulT( t.rotation, u.rotation );
	v.position = rpcMulT( t.rotation, u.position - t.position );
	return v;
}

//--------------------------------------------------------------------------------------------------
inline const rpcHalfSpace rpcMulT( const rpcTransform& tx, const rpcHalfSpace& p )
{
	rpcVec3 origin = p.normal * p.distance;
	origin = rpcMulT( tx, origin );
	rpcVec3 n = rpcMulT( tx.rotation, p.normal );
	return rpcHalfSpace( n, rpcDot( origin, n ) );
}

//--------------------------------------------------------------------------------------------------
inline void rpcIdentity( rpcTransform& tx )
{
	rpcIdentity( tx.position );
	rpcIdentity( tx.rotation );
}

//--------------------------------------------------------------------------------------------------
inline const rpcTransform rpcInverse( const rpcTransform& tx )
{
	rpcTransform inverted;
	inverted.rotation = rpcTranspose(tx.rotation);
	inverted.position = rpcMul(inverted.rotation, -tx.position);
	return inverted;
}