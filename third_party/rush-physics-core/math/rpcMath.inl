//--------------------------------------------------------------------------------------------------
// rpcMath.inl
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
inline r32 rpcInvert( r32 a )
{
	return a != 0.0f ? 1.0f / a : 0.0f;
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcSign( r32 a )
{
	if ( a >= r32( 0.0 ) )
	{
		return r32( 1.0 );
	}

	else
	{
		return r32( -1.0 );
	}
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcAbs( r32 a )
{
	if ( a < r32( 0.0 ) )
		return -a;

	return a;
}

//--------------------------------------------------------------------------------------------------
inline i32 rpcMin( i32 a, i32 b )
{
	if ( a < b )
		return a;

	return b;
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcMin( r32 a, r32 b )
{
	if ( a < b )
		return a;

	return b;
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcMax( r32 a, r32 b )
{
	if ( a > b )
		return a;

	return b;
}

//--------------------------------------------------------------------------------------------------
inline i32 rpcMax( i32 a, i32 b )
{
	if ( a > b )
		return a;

	return b;
}

//--------------------------------------------------------------------------------------------------
inline u8 rpcMax( u8 a, u8 b )
{
	if ( a > b )
		return a;

	return b;
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcClamp01( r32 val )
{
	if ( val >= r32( 1.0 ) )
		return 1.0;

	if ( val <= r32( 0.0 ) )
		return 0.0;

	return val;
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcClamp( r32 min, r32 max, r32 a )
{
	if ( a < min )
		return min;

	if ( a > max )
		return max;

	return a;
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcLerp( r32 a, r32 b, r32 t )
{
	return a * (r32( 1.0 ) - t) + b * t;
}

//--------------------------------------------------------------------------------------------------
inline const rpcVec3 rpcLerp( const rpcVec3& a, const rpcVec3& b, r32 t )
{
	return a * (r32( 1.0 ) - t) + b * t;
}

//--------------------------------------------------------------------------------------------------
inline r32 rpcRandomFloat( r32 l, r32 h )
{
	r32 a = r32( rand( ) );
	a /= r32( RAND_MAX );
	a = (h - l) * a + l;

	return a;
}

//--------------------------------------------------------------------------------------------------
inline i32 rpcRandomInt( i32 low, i32 high )
{
	return (rand( ) % (high - low + 1) + low);
}
