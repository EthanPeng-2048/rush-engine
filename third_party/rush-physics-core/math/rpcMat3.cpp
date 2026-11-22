//--------------------------------------------------------------------------------------------------
/**
@file	rpcMat3.cpp

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

#include <cassert>
#include "rpcMat3.h"

//--------------------------------------------------------------------------------------------------
// rpcMat3
//--------------------------------------------------------------------------------------------------
rpcMat3::rpcMat3( )
{
}

//--------------------------------------------------------------------------------------------------
rpcMat3::rpcMat3( r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i )
	: ex( a, b, c )
	, ey( d, e, f )
	, ez( g, h, i )
{
}

//--------------------------------------------------------------------------------------------------
rpcMat3::rpcMat3( const rpcVec3& _x, const rpcVec3& _y, const rpcVec3& _z )
	: ex( _x )
	, ey( _y )
	, ez( _z )
{
}

//--------------------------------------------------------------------------------------------------
void rpcMat3::Set( r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i )
{
	ex.Set( a, b, c );
	ey.Set( d, e, f );
	ez.Set( g, h, i );
}

//--------------------------------------------------------------------------------------------------
void rpcMat3::Set( const rpcVec3& axis, r32 angle )
{
	r32 s = std::sin( angle );
	r32 c = std::cos( angle );
	r32 x = axis.x;
	r32 y = axis.y;
	r32 z = axis.z;
	r32 xy = x * y;
	r32 yz = y * z;
	r32 zx = z * x;
	r32 t = r32( 1.0 ) - c;

	Set(
		x * x * t + c, xy * t + z * s, zx * t - y * s,
		xy * t - z * s, y * y * t + c, yz * t + x * s,
		zx * t + y * s, yz * t - x * s, z * z * t + c
		);
}

//--------------------------------------------------------------------------------------------------
void rpcMat3::SetRows( const rpcVec3& x, const rpcVec3& y, const rpcVec3& z )
{
	ex = x;
	ey = y;
	ez = z;
}

//--------------------------------------------------------------------------------------------------
rpcMat3& rpcMat3::operator=( const rpcMat3& rhs )
{
	ex = rhs.ex;
	ey = rhs.ey;
	ez = rhs.ez;

	return *this;
}

//--------------------------------------------------------------------------------------------------
rpcMat3& rpcMat3::operator*=( const rpcMat3& rhs )
{
	*this = *this * rhs;

	return *this;
}

//--------------------------------------------------------------------------------------------------
rpcMat3& rpcMat3::operator*=( r32 f )
{
	ex *= f;
	ey *= f;
	ez *= f;

	return *this;
}


//--------------------------------------------------------------------------------------------------
rpcMat3& rpcMat3::operator+=( const rpcMat3& rhs )
{
	ex += rhs.ex;
	ey += rhs.ey;
	ez += rhs.ez;

	return *this;
}

//--------------------------------------------------------------------------------------------------
rpcMat3& rpcMat3::operator-=( const rpcMat3& rhs )
{
	ex -= rhs.ex;
	ey -= rhs.ey;
	ez -= rhs.ez;

	return *this;
}

//--------------------------------------------------------------------------------------------------
rpcVec3& rpcMat3::operator[]( u32 index )
{
	switch ( index )
	{
	case 0:
		return ex;
	case 1:
		return ey;
	case 2:
		return ez;
	default:
		assert( false );
		return ex;
	}
}

//--------------------------------------------------------------------------------------------------
const rpcVec3& rpcMat3::operator[]( u32 index ) const
{
	switch ( index )
	{
	case 0:
		return ex;
	case 1:
		return ey;
	case 2:
		return ez;
	default:
		assert( false );
		return ex;
	}
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcMat3::Column0( ) const
{
	return rpcVec3( ex.x, ey.x, ez.x );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcMat3::Column1( ) const
{
	return rpcVec3( ex.y, ey.y, ez.y );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcMat3::Column2( ) const
{
	return rpcVec3( ex.z, ey.z, ez.z );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcMat3::operator*( const rpcVec3& rhs ) const
{
	return rpcVec3(
		ex.x * rhs.x + ey.x * rhs.y + ez.x * rhs.z,
		ex.y * rhs.x + ey.y * rhs.y + ez.y * rhs.z,
		ex.z * rhs.x + ey.z * rhs.y + ez.z * rhs.z
		);
}

//--------------------------------------------------------------------------------------------------
const rpcMat3 rpcMat3::operator*( const rpcMat3& rhs ) const
{
	return rpcMat3(
		( *this * rhs.ex ),
		( *this * rhs.ey ),
		( *this * rhs.ez )
		);
}

//--------------------------------------------------------------------------------------------------
const rpcMat3 rpcMat3::operator*( r32 f ) const
{
	return rpcMat3( ex * f, ey * f, ez * f );
}

//--------------------------------------------------------------------------------------------------
const rpcMat3 rpcMat3::operator+( const rpcMat3& rhs ) const
{
	return rpcMat3( ex + rhs.ex, ey + rhs.ey, ez + rhs.ez );
}

//--------------------------------------------------------------------------------------------------
const rpcMat3 rpcMat3::operator-( const rpcMat3& rhs ) const
{
	return rpcMat3( ex - rhs.ex, ey - rhs.ey, ez - rhs.ez );
}
