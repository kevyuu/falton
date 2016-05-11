//
// Created by Kevin Yu on 4/14/16.
//

#ifndef FALTON_FTPHYSICSSYSTEM_H
#define FALTON_FTPHYSICSSYSTEM_H

#include <falton/math/precision.h>
#include <falton/physics/dynamic/ftBody.h>
#include <falton/physics/ftContactSolver.h>
#include <falton/physics/collision/ftContact.h>
#include <falton/physics/dynamic/ftContactConstraint.h>
#include <falton/physics/dynamic/ftIslandSystem.h>

struct ftBodyDef;
struct ftColliderDef;
struct ftCollider;
class ftPinJoint;

class ftPhysicsSystem {

public:

    void init(ftVector2 gravity);
    void shutdown();

    ftBody* createBody(const ftBodyDef& bodyDef);
    ftCollider* createCollider(const ftColliderDef& colliderDef);
    ftPinJoint* createJoint(ftBody* bodyA, ftBody* bodyB, ftVector2 anchorPoint);

    void destroyBody(ftBody* body);
    void destroyCollider(ftCollider* collider);

    void step(real dt);

private:
    ftBodyBuffer m_staticBodies;
    ftBodyBuffer m_kinematicBodies;
    ftBodyBuffer m_dynamicBodies;

    ftContactBuffer m_contactBuffer;

    ftContactSolver m_contactSolver;
    ftCollisionSystem m_collisionSystem;
    ftIslandSystem m_islandSystem;

    ftBroadphaseSystem *m_broadphase;

    ftChunkArray<ftPinJoint*> m_joints;

    ftVector2 m_gravity;

    void integrateVelocity(real dt);
    void integratePosition(real dt);

    static void beginContactListener(ftContact* contact, void* data);
    static void updateContactListener(ftContact* contact, void* data);
    static void endContactListener(ftContact* contact, void* data);
    static bool collisionFilter(void* userdataA, void* userdataB);

};


#endif //FALTON_FTPHYSICSSYSTEM_H
