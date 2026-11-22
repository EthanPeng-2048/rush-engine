//--------------------------------------------------------------------------------------------------
/**
@file	rpc.h

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

/**
Error contract (L1 vs L2):
  - L1 (this library) never uses exceptions and never returns Result objects.
    Invalid input (null pointers, foreign handles, non-finite values) is a
    caller contract violation: Debug builds assert; Release builds make the
    operation a no-op where possible. No crash-by-design, no hidden retries.
  - L2 adapters (rush-engine RushPhysicsCore) own validation and surface
    errors as rush::Result / EX_FAIL to gameplay code.
  - Query misses are not errors: RayHit.hit stays false, ReportShape is
    simply not called.
  - Contact listeners must be cheap: no allocations, no body create/destroy
    inside BeginContact/EndContact — defer that work to the caller's stage.
*/

#ifndef RPC_H
#define RPC_H

#include "common/rpcTypes.h"
#include "scene/rpcScene.h"
#include "dynamics/rpcBody.h"
#include "collision/rpcBox.h"
#include "math/rpcVec3.h"
#include "math/rpcMat3.h"
#include "math/rpcQuaternion.h"
#include "math/rpcTransform.h"
#include "debug/rpcRender.h"

#endif // RPC_H
