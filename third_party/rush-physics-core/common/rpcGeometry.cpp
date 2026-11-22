//--------------------------------------------------------------------------------------------------
/**
@file	rpcGeometry.cpp

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

#include "rpcGeometry.h"

//--------------------------------------------------------------------------------------------------
// rpcHalfSpace
//--------------------------------------------------------------------------------------------------
rpcHalfSpace::rpcHalfSpace( )
{
}

//--------------------------------------------------------------------------------------------------
rpcHalfSpace::rpcHalfSpace( const rpcVec3& n, r32 d )
	: normal( n )
	, distance( d )
{
}

//--------------------------------------------------------------------------------------------------
void rpcHalfSpace::Set( const rpcVec3& a, const rpcVec3& b, const rpcVec3& c )
{
	normal = rpcNormalize( rpcCross( b - a, c - a ) );
	distance = rpcDot( normal, a );
}

//--------------------------------------------------------------------------------------------------
void rpcHalfSpace::Set( const rpcVec3& n, const rpcVec3& p )
{
	normal = rpcNormalize( n );
	distance = rpcDot( normal, p );
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcHalfSpace::Origin( ) const
{
	return normal * distance;
}

//--------------------------------------------------------------------------------------------------
r32 rpcHalfSpace::Distance( const rpcVec3& p ) const
{
	return rpcDot( normal, p ) - distance;
}

//--------------------------------------------------------------------------------------------------
const rpcVec3 rpcHalfSpace::Projected( const rpcVec3& p ) const
{
	return p - normal * Distance( p );
}
