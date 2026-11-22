//--------------------------------------------------------------------------------------------------
/**
@file	rpcScene.h

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

#ifndef RPCSCENE_H
#define RPCSCENE_H

#include <stdio.h>

#include "../common/rpcSettings.h"
#include "../common/rpcMemory.h"
#include "../dynamics/rpcContactManager.h"

//--------------------------------------------------------------------------------------------------
// rpcScene
//--------------------------------------------------------------------------------------------------
class rpcBody;
struct rpcBodyDef;
struct rpcContactConstraint;
class rpcRender;
struct rpcIsland;

// This listener is used to gather information about two shapes colliding. This
// can be used for game logic and sounds. Physics objects created in these
// callbacks will not be reported until the following frame. These callbacks
// can be called frequently, so make them efficient.
class rpcContactListener
{
public:
	virtual ~rpcContactListener( )
	{
	}

	virtual void BeginContact( const rpcContactConstraint *contact ) = 0;
	virtual void EndContact( const rpcContactConstraint *contact ) = 0;
};

// This class represents general queries for points, AABBs and Raycasting.
// ReportShape is called the moment a valid shape is found. The return
// value of ReportShape controls whether to continue or stop the query.
// By returning only true, all shapes that fulfill the query will be re-
// ported.
class rpcQueryCallback
{
public:
	virtual ~rpcQueryCallback( )
	{
	}

	virtual bool ReportShape( rpcBox *box ) = 0;
};

class rpcScene
{
public:
	rpcScene( r32 dt, const rpcVec3& gravity = rpcVec3( r32( 0.0 ), r32( -9.8 ), r32( 0.0 ) ), i32 iterations = 20 );
	~rpcScene( );

	// Run the simulation forward in time by dt (fixed timestep). Variable
	// timestep is not supported.
	void Step( );

	// Construct a new rigid body. The BodyDef can be reused at the user's
	// discretion, as no reference to the BodyDef is kept.
	rpcBody* CreateBody( const rpcBodyDef& def );

	// Frees a body, removes all shapes associated with the body and frees
	// all shapes and contacts associated and attached to this body.
	void RemoveBody( rpcBody* body );
	void RemoveAllBodies( );

	// Enables or disables rigid body sleeping. Sleeping is an effective CPU
	// optimization where bodies are put to sleep if they don't move much.
	// Sleeping bodies sit in memory without being updated, until the are
	// touched by something that wakes them up. The default is enabled.
	void SetAllowSleep( bool allowSleep );

	// Increasing the iteration count increases the CPU cost of simulating
	// Scene.Step(). Decreasing the iterations makes the simulation less
	// realistic (convergent). A good iteration number range is 5 to 20.
	// Only positive numbers are accepted. Non-positive and negative
	// inputs set the iteration count to 1.
	void SetIterations( i32 iterations );

	// Friction occurs when two rigid bodies have shapes that slide along one
	// another. The friction force resists this sliding motion.
	void SetEnableFriction( bool enabled );

	// Render the scene with an interpolated time between the last frame and
	// the current simulation step.
	void Render( rpcRender* render ) const;

	// Gets and sets the global gravity vector used during integration
	const rpcVec3 GetGravity( ) const;
	void SetGravity( const rpcVec3& gravity );

	// Removes all bodies from the scene.
	void Shutdown( );

	// Sets the listener to report collision start/end. Provides the user
	// with a pointer to an rpcContactConstraint. The rpcContactConstraint
	// holds pointers to the two shapes involved in a collision, and the
	// two bodies connected to each shape. The rpcContactListener will be
	// called very often, so it is recommended for the funciton to be very
	// efficient. Provide a NULL pointer to remove the previously set
	// listener.
	void SetContactListener( rpcContactListener* listener );

	// Query the world to find any shapes that can potentially intersect
	// the provided AABB. This works by querying the broadphase with an
	// AAABB -- only *potential* intersections are reported. Perhaps the
	// user might use lmDistance as fine-grained collision detection.
	void QueryAABB( rpcQueryCallback *cb, const rpcAABB& aabb ) const;

	// Query the world to find any shapes intersecting a world space point.
	void QueryPoint( rpcQueryCallback *cb, const rpcVec3& point ) const;

	// Query the world to find any shapes intersecting a ray.
	void RayCast( rpcQueryCallback *cb, rpcRaycastData& rayCast ) const;

	// Dump all rigid bodies and shapes into a log file. The log can be
	// used as C++ code to re-create an initial scene setup. Contacts
	// are *not* logged, meaning any cached resolution solutions will
	// not be saved to the log file. This means the log file will be most
	// accurate when dumped upon scene initialization, instead of mid-
	// simulation.
	void Dump( FILE* file ) const;

private:
	// Step pipeline stages (rpc-refactor.md D6): one concern per method.
	void SolveIslands( );	// seed loop: BuildIsland per seed, then Initialize/Solve
	void SyncAndFind( );	// SynchronizeProxies + FindNewContacts + clear forces

	// DFS the constraint graph from seed into the working island. Stack must
	// hold at least m_bodyCount entries.
	void BuildIsland( rpcBody* seed, rpcIsland& island, rpcBody** stack, i32 stackSize );

	rpcContactManager m_contactManager;
	rpcPagedAllocator m_boxAllocator;

	i32 m_bodyCount;
	rpcBody* m_bodyList;

	rpcStack m_stack;
	rpcHeap m_heap;

	rpcVec3 m_gravity;
	r32 m_dt;
	i32 m_iterations;

	bool m_newBox;
	bool m_allowSleep;
	bool m_enableFriction;

	friend class rpcBody;
};

#endif // RPCSCENE_H
