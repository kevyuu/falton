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

struct ftBodyDef;
struct ftColliderDef;
struct ftCollider;

class ftIslandSystem;

class ftPhysicsSystem {

public:

    void init(ftVector2 gravity);
    void shutdown();

    ftBody* createBody(const ftBodyDef& bodyDef);
    ftCollider* createCollider(const ftColliderDef& colliderDef);

    void destroyBody(ftBody* body);
    void destroyCollider(ftCollider* collider);

    void step(real dt);

private:
    ftBodyBuffer staticBodies;
    ftBodyBuffer kinematicBodies;
    ftBodyBuffer dynamicBodies;

    ftContactConstraint *contactConstraint;
    ftContactBuffer contactBuffer;

    ftContactSolver *contactSolver;
    ftCollisionSystem *collisionSystem;
    ftIslandSystem *islandSystem;
    ftBroadPhase *broadphase;

    ftVector2 gravity;

    void integrateVelocity(real dt);
    void integratePosition(real dt);

    static void beginContactListener(ftContact* contact, void* data);
    static void updateContactListener(ftContact* contact, void* data);
    static void endContactListener(ftContact* contact, void* data);

};


#endif //FALTON_FTPHYSICSSYSTEM_H
