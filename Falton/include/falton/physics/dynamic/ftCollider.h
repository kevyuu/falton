//
// Created by Kevin Yu on 12/29/15.
//

#ifndef FALTON_FTCOLLIDER_H
#define FALTON_FTCOLLIDER_H

#include "falton/math/math.h"
#include "falton/physics/shape/ftShape.h"
#include "falton/physics/Collision/ftCollisionSystem.h"

struct ftBody;

struct ftColliderDef {
    ftBody *body = nullptr;

    ftVector2 position;
    real orientation = 0;

    real restitution = 0;
    real friction = 0.2;

    ftShape *shape = nullptr;
};

struct ftCollider {

public:

    ftBody *body = nullptr;

    //position and orientation relative to body
    ftTransform transform;

    real friction;
    real restitution;
    ftColHandle collisionHandle;

    ftShape* shape = nullptr;
    ftCollider* next = nullptr;

    friend class ftPhysicsSystem;
    friend class ftContactSolver;
};


#endif //FALTON_FTCOLLIDER_H
