#include "btPhysicsResolver.h"

btPhysicsResolver::btPhysicsResolver() { }
btPhysicsResolver::~btPhysicsResolver() { }

void btPhysicsResolver::overlap(btPairCachingGhostObject* ghostObject, btCollisionWorld* collisionWorld, OverlapResultCallback& resultCallback) {
	
	btVector3 minAabb, maxAabb;
	ghostObject->getCollisionShape()->getAabb(ghostObject->getWorldTransform(), minAabb, maxAabb);
	
	collisionWorld->getBroadphase()->setAabb(ghostObject->getBroadphaseHandle(), minAabb,  maxAabb, collisionWorld->getDispatcher()); 
	collisionWorld->getDispatcher()->dispatchAllCollisionPairs(ghostObject->getOverlappingPairCache(), collisionWorld->getDispatchInfo(), collisionWorld->getDispatcher());

	btManifoldArray manifoldArray;

	const int count = ghostObject->getOverlappingPairCache()->getNumOverlappingPairs();

	for (int i = 0; i < count; i++) {

		manifoldArray.resize(0);

		btBroadphasePair* collisionPair = &ghostObject->getOverlappingPairCache()->getOverlappingPairArray()[i];
		btCollisionObject* obj0 = static_cast<btCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
		btCollisionObject* obj1 = static_cast<btCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

		if ((obj0 && !obj0->hasContactResponse()) || (obj1 && !obj1->hasContactResponse()))
			continue;
		
		if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);
		
		for (int j = 0; j < manifoldArray.size(); j++) {

			btPersistentManifold* manifold = manifoldArray[j];

			for (int p = 0; p < manifold->getNumContacts(); p++) {

				//btCollisionObjectWrapper tmpOb(0, childCollisionShape, m_collisionObject, childWorldTrans, -1, i);

				//resultCallback.needsCollision();

				btManifoldPoint& pt = manifold->getContactPoint(p);

				resultCallback.addSingleResult(pt);
			}

			/*
			for (int p = 0; p < manifold->getNumContacts(); p++) {

				const btManifoldPoint&pt = manifold->getContactPoint(p);

				btScalar dist = pt.getDistance();

				if (dist < 0.0)
				{
					if (dist < maxPen)
					{
						maxPen = dist;
						m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??

					}
					m_currentPosition += pt.m_normalWorldOnB * directionSign * dist * btScalar(0.2);
					penetration = true;
				} else {
					//printf("touching %f\n", dist);
				}
			}
			
			manifold->clearManifold();
			*/
		}
	}
}