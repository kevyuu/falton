//
// Created by Kevin Yu on 12/26/15.
//

#ifndef FALTON_FTSPACE_H
#define FALTON_FTSPACE_H

#include "ftDef.h"
#include "ftBody.h"
#include "ftCollider.h"
#include "falton/physics/Collision/ftCollisionSystem.h"
#include "falton/physics/Collision/BroadPhase/ftNSquaredBroadphase.h"
#include "falton/physics/ftContactSolver.h"

class ftBody;
class ftPinJoint;

typedef void (*ftBodyIterFunc)(ftBody* body, void *data);

class ftSpace {
public:

    ftCollisionSystem collisionSystem;
    ftNSquaredBroadphase nSquaredBroadphase;
    ftContactSolver *contactSolver;

    ftVector2 gravity;

    ftSpace(ftVector2 gravity);
    ~ftSpace();

    ftBody* createBody(const ftBodyDef& bodyDef);
    void iterateBody(ftBodyIterFunc iterFunc, void *data);

    void destroyBody(ftBody* body);

    ftCollider* createCollider(const ftColliderDef& colliderDef);
    void destroyCollider(ftCollider* collider);

    ftPinJoint* createJoint(ftBody* bodyA, ftBody* bodyB, ftVector2 anchorPoint);

    void step(real dt);

private:
    ftBody* staticBodies;
    ftBody* kinematicBodies;
    ftBody* dynamicBodies;

};

static void beginContactListener(void *userDataA, void *userDataB, ftContact *contact, void *data);
static void updateContactListener(void *userDataA, void *userDataB, ftContact *contact, void *data);
static void endContactListener(void *userDataA, void *userDataB, ftContact *contact, void *data);


#endif //FALTON_FTSPACE_H
