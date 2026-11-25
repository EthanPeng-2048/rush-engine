//--------------------------------------------------------------------------------------------------
/**
@file	rpcSettings.h

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

#ifndef RPCSETTINGS_H
#define RPCSETTINGS_H

#include "../math/rpcMath.h"
//--------------------------------------------------------------------------------------------------
// Internal Implementation Constants (do not change unless you know what you're doing)
//--------------------------------------------------------------------------------------------------
#define RPC_SLEEP_LINEAR r32( 0.01 )

#define RPC_SLEEP_ANGULAR r32( (3.0 / 180.0) * rpcPI )

#define RPC_SLEEP_TIME r32( 0.5 )

#define RPC_BAUMGARTE r32( 0.2 )

#define RPC_PENETRATION_SLOP r32( 0.05 )

// Leaf margin added to every proxy AABB. Larger = fewer re-insertions but more
// false overlaps (and a tree whose nodes overlap each other, which hurts every
// query: a query must descend into every node the ray/AABB touches). 0.2 m
// still covers a car's per-substep travel at 480 Hz (~0.15 m at 250 km/h).
#define RPC_AABB_MARGIN r32( 0.2 )

// Positional (Baumgarte) correction budget per step, in metres. Without this
// cap a deep overlap — a pile-up, a car spawned inside another car — turns into
// a correction *velocity* of depth/dt (hundreds of m/s at 480 Hz) and launches
// the bodies. Box2D caps the same term with b2_maxLinearCorrection.
#define RPC_MAX_LINEAR_CORRECTION r32( 0.2 )

// Absolute velocity guards. A racing sim never needs these numbers; they exist
// so a numerically degenerate step (bad manifold, huge restitution impulse)
// cannot turn into a multi-kilometre launch or permanent NaN.
#define RPC_MAX_LINEAR_VELOCITY r32( 150.0 )
#define RPC_MAX_ANGULAR_VELOCITY r32( 80.0 )

// Contact budget between two DYNAMIC bodies. Composite bodies (a car is made of
// ~11 boxes) otherwise generate up to boxes(A) x boxes(B) manifolds per pair —
// 121 for two cars — and every one of them is narrowphase + solver work. Capping
// the pair keeps a pile-up's cost proportional to the number of touching pairs
// instead of to the product of their box counts.
// Dynamic-vs-static contacts (ground, walls) are never capped: they carry the
// load path that keeps a car on the road.
#define RPC_MAX_CONTACTS_PER_DYNAMIC_PAIR 12



#endif // RPCSETTINGS_H
