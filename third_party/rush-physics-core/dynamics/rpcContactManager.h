//--------------------------------------------------------------------------------------------------
/**
@file	rpcContactManager.h

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

#ifndef RPCCONTACTMANAGER_H
#define RPCCONTACTMANAGER_H

#include "../common/rpcTypes.h"
#include "../broadphase/rpcBroadPhase.h"
#include "../common/rpcMemory.h"

//--------------------------------------------------------------------------------------------------
// rpcContactManager
//--------------------------------------------------------------------------------------------------
struct rpcContactConstraint;
class rpcContactListener;
struct rpcBox;
class rpcBody;
class rpcRender;
class rpcStack;

class rpcContactManager
{
public:
	rpcContactManager( rpcStack* stack );

	// Add a new contact constraint for a pair of objects
	// unless the contact constraint already exists
	void AddContact( rpcBox *A, rpcBox *B );

	// Has broadphase find all contacts and call AddContact on the
	// ContactManager for each pair found
	void FindNewContacts( void );

	// Remove a specific contact
	void RemoveContact( rpcContactConstraint *contact );

	// Remove all contacts from a body
	void RemoveContactsFromBody( rpcBody *body );
	void RemoveFromBroadphase( rpcBody *body );

	// Remove contacts without broadphase overlap
	// Solves contact manifolds
	void TestCollisions( void );
	static void SolveCollision( void* param );

	void RenderContacts( rpcRender* debugDrawer ) const;

private:
	rpcContactConstraint* m_contactList;
	i32 m_contactCount;
	rpcStack* m_stack;
	rpcPagedAllocator m_allocator;
	rpcBroadPhase m_broadphase;
	rpcContactListener *m_contactListener;

	friend class rpcBroadPhase;
	friend class rpcScene;
	friend struct rpcBox;
	friend class rpcBody;
};

#endif // RPCCONTACTMANAGER_H
