#ifndef BT_PHYSICS_RESOLVER_H
#define BT_PHYSICS_RESOLVER_H

#include "LinearMath/btVector3.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

class btManifoldPoint;
class btCollisionObject;
class btCollisionWorld;
class btPairCachingGhostObject;

ATTRIBUTE_ALIGNED16(class) btPhysicsResolver {

public:

	btPhysicsResolver();
	~btPhysicsResolver();

	struct OverlapResultCallback
	{
		short int m_collisionFilterGroup;
		short int m_collisionFilterMask;
		
		OverlapResultCallback() :
			m_collisionFilterGroup(btBroadphaseProxy::DefaultFilter),
			m_collisionFilterMask(btBroadphaseProxy::AllFilter)
		{
		}

		virtual ~OverlapResultCallback()
		{
		}
		
		virtual bool needsCollision(btBroadphaseProxy* proxy0) const
		{
			bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}

		virtual btScalar debug(btScalar value) = 0;
		virtual	btScalar addSingleResult(btManifoldPoint& cp) = 0;
	};

	void overlap(btPairCachingGhostObject* ghostObject, btCollisionWorld* collisionWorld, OverlapResultCallback& resultCallback);
};

#endif // BT_PHYSICS_RESOLVER_H