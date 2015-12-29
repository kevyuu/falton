//
// Created by Kevin Yu on 12/26/15.
//

#ifndef FALTON_FTDEF_H
#define FALTON_FTDEF_H

#include "falton/math/math.h"

struct ftBodyDef {
    ftVector2 position;
    ftVector2 velocity;

    real orientation;
    real angular_velocity;

    real mass;

    real rotation_inertia;
};

struct ftColliderDef {
    ftVector2 position;

    real restitution;
    real friction;
};



#endif //FALTON_FTDEF_H
