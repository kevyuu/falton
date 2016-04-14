//
// Created by Kevin Yu on 12/26/15.
//

#ifndef FALTON_FTDEF_H
#define FALTON_FTDEF_H

#include "falton/math/math.h"
#include "falton/physics/ftBody.h"

class ftCollider;
class ftShape;



struct ftColliderDef {
    ftBody *body = nullptr;

    ftVector2 position;
    real orientation = 0;

    real restitution = 0;
    real friction = 0.2;

    ftShape *shape = nullptr;
};



#endif //FALTON_FTDEF_H
