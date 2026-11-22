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

#endif // RPCSETTINGS_H
