//--------------------------------------------------------------------------------------------------
/**
@file	rpcMat3.h

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

#ifndef RPCMAT3_H
#define RPCMAT3_H

#include <cstring>	// memset
#include "../common/rpcTypes.h"
#include "rpcVec3.h"

//--------------------------------------------------------------------------------------------------
// rpcMat3
//--------------------------------------------------------------------------------------------------
struct rpcMat3
{
	rpcVec3 ex;
	rpcVec3 ey;
	rpcVec3 ez;

	rpcMat3( );
	rpcMat3( r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i );
	rpcMat3( const rpcVec3& _x, const rpcVec3& _y, const rpcVec3& _z );

	void Set( r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i );
	void Set( const rpcVec3& axis, r32 angle );
	void SetRows( const rpcVec3& x, const rpcVec3& y, const rpcVec3& z );

	rpcMat3& operator=( const rpcMat3& rhs );
	rpcMat3& operator*=( const rpcMat3& rhs );
	rpcMat3& operator*=( r32 f );
	rpcMat3& operator+=( const rpcMat3& rhs );
	rpcMat3& operator-=( const rpcMat3& rhs );

	rpcVec3& operator[]( u32 index );
	const rpcVec3& operator[]( u32 index ) const;
	const rpcVec3 Column0( ) const;
	const rpcVec3 Column1( ) const;
	const rpcVec3 Column2( ) const;

	const rpcVec3 operator*( const rpcVec3& rhs ) const;
	const rpcMat3 operator*( const rpcMat3& rhs ) const;
	const rpcMat3 operator*( r32 f ) const;
	const rpcMat3 operator+( const rpcMat3& rhs ) const;
	const rpcMat3 operator-( const rpcMat3& rhs ) const;
};

#include "rpcMat3.inl"

#endif // RPCMAT3_H
