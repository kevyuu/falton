//
// Created by Kevin Yu on 12/6/15.
//

#ifndef FALTON_RIGIDBODY_H
#define FALTON_RIGIDBODY_H

#include "falton/math/math.h"


class ftCollider;
struct ftBodyDef;

enum ftBodyType {
    STATIC,
    KINEMATIC,
    DYNAMIC
};

struct ftBodyDef {
    ftVector2 position;
    ftVector2 velocity;
    ftVector2 centerOfMass;

    ftBodyType bodyType = STATIC;

    real mass;
    real moment;

    real orientation = 0;
    real angularVelocity = 0;
};

class ftBody {
public:

    ftTransform transform;

    ftVector2 velocity;

    ftVector2 forceAccum;

    ftVector2 centerOfMass;

    ftBodyType bodyType;

    real torqueAccum;

    real angularVelocity;

    real mass;
    real inverseMass;
    real moment;
    real inverseMoment;

    ftBody* next;
    ftBody* prev;

    ftCollider* colliders;

    ftBody(const ftBodyDef& bodyDef);
    void init(const ftBodyDef& bodyDef);
    void addCollider(ftCollider *collider);

};

#endif //FALTON_RIGIDBODY_H
