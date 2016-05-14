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

typedef void (*ftBodyIterFunc) (ftBody* body, void* data);

class ftPhysicsSystem {

public:

    void init(ftVector2 gravity);
    void shutdown();

    ftBody* createBody(const ftBodyDef& bodyDef);
    ftCollider* createCollider(const ftColliderDef& colliderDef);
    ftPinJoint* createJoint(ftBody* bodyA, ftBody* bodyB, ftVector2 anchorPoint);

    void destroyBody(ftBody* body);
    void destroyCollider(ftCollider* collider);

    void iterateBody(ftBodyIterFunc iterFunc, void* data);
    void iterateStaticBody(ftBodyIterFunc iterFunc, void* data);
    void iterateKinematicBody(ftBodyIterFunc iterFunc, void* data);
    void iterateDynamicBody(ftBodyIterFunc iterFunc, void* data);

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

    static constexpr float SLEEP_TIME_THRESHOLD = 0.5f;
    static constexpr float SLEEP_LIN_SPEED_THRESHOLD = 0.08f;
    static constexpr float SLEEP_ANG_SPEED_THRESHOLD = (2.0f / 180.0f * PI);

    void integrateVelocity(real dt);
    void integratePosition(real dt);

    void updateBodiesActivation(real dt);
    void updateBodyActivation(ftBody* body, real dt);

    static void beginContactListener(ftContact* contact, void* data);
    static void updateContactListener(ftContact* contact, void* data);
    static void endContactListener(ftContact* contact, void* data);

};


#endif //FALTON_FTPHYSICSSYSTEM_H
