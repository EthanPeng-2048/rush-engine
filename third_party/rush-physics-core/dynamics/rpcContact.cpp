//--------------------------------------------------------------------------------------------------
/**
@file	rpcContact.cpp

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

#include "rpcContact.h"

//--------------------------------------------------------------------------------------------------
// rpcContact
//--------------------------------------------------------------------------------------------------
void rpcManifold::SetPair( rpcBox *a, rpcBox *b )
{
	A = a;
	B = b;

	sensor = A->sensor || B->sensor;
}

// Generate contact information
void rpcContactConstraint::SolveCollision( void )
{
	manifold.contactCount = 0;

	rpcBoxtoBox( &manifold, A, B );

	if ( manifold.contactCount > 0 )
	{
		if ( m_flags & eColliding )
			m_flags |= eWasColliding;

		else
			m_flags |= eColliding;
	}

	else
	{
		if ( m_flags & eColliding )
		{
			m_flags &= ~eColliding;
			m_flags |= eWasColliding;
		}

		else
			m_flags &= ~eWasColliding;
	}
}
