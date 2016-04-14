//
// Created by Kevin Yu on 12/29/15.
//

#ifndef FALTON_FTCOLLIDER_H
#define FALTON_FTCOLLIDER_H

#include "falton/math/math.h"
#include "falton/physics/shape/ftShape.h"
#include "falton/physics/ftDef.h"
#include "falton/physics/Collision/ftCollisionSystem.h"

class ftCollider {
public:

    ftBody *body;

    //position and orientation relative to body
    ftTransform transform;

    real friction;
    real restitution;
    ColHandle collisionHandle;

    ftShape* shape;
    ftCollider* next;

    ftCollider(const ftColliderDef& colliderDef);
};


#endif //FALTON_FTCOLLIDER_H
