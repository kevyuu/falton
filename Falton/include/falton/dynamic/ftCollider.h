//
// Created by Kevin Yu on 12/29/15.
//
#pragma once

#include <falton/math.h>
#include <falton/shape/ftShape.h>
#include <falton/collision/ftCollisionSystem.h>

struct ftBody;

struct ftCollider {

    ftBody *body = nullptr;

    //position and orientation relative to body
    ftTransform transform;

    real friction = 0.2f;
    real restitution = 0;

    uint32 group = 0;
    uint32 category = 0xFFFF;
    uint32 mask = 0xFFFF;

    ftColHandle collisionHandle;

    ftShape* shape = nullptr;
    ftCollider* next = nullptr;

};
