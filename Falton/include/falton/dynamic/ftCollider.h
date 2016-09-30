//
// Created by Kevin Yu on 12/29/15.
//

#ifndef FALTON_FTCOLLIDER_H
#define FALTON_FTCOLLIDER_H

#include "falton/math/math.h"
#include "falton/physics/shape/ftShape.h"
#include "falton/physics/Collision/ftCollisionSystem.h"

struct ftBody;

struct ftCollider {

public:

    ftBody *body = nullptr;

    //position and orientation relative to body
    ftTransform transform;

    real friction = 0.2;
    real restitution = 0;

    uint32 group = 0;
    uint32 category = 0xFFFF;
    uint32 mask = 0xFFFF;

    ftColHandle collisionHandle;

    ftShape* shape = nullptr;
    ftCollider* next = nullptr;

    friend class ftPhysicsSystem;
    friend class ftConstraintSolver;
};


#endif //FALTON_FTCOLLIDER_H
