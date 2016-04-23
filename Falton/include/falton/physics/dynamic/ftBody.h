//
// Created by Kevin Yu on 12/6/15.
//

#ifndef FALTON_RIGIDBODY_H
#define FALTON_RIGIDBODY_H

#include "falton/math/math.h"

struct ftCollider;
struct ftBody;
struct ftContact;

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

    real mass = 0;
    real moment = 0;

    real orientation = 0;
    real angularVelocity = 0;
};

struct ftContactEdge {
    ftBody* other = nullptr;
    ftContact* contact = nullptr;
    ftContactEdge* prev = nullptr;
    ftContactEdge* next = nullptr;
};

struct ftBody {
public:
    ftTransform transform;

    ftVector2 velocity;

    ftVector2 forceAccum;

    ftVector2 centerOfMass;

    ftBodyType bodyType;

    real torqueAccum = 0;

    real angularVelocity;

    real mass;
    real inverseMass;
    real moment;
    real inverseMoment;

    ftCollider* colliders = nullptr;

    ftContactEdge* contactList = nullptr;

    int32 islandId;

    friend class ftPhysicsSystem;
    friend class ftIslandSystem;
    friend class ftContactSolver;

};

class ftBodyBuffer {

private:
    struct ftBodyElem {
        ftBody body;
        ftBodyElem* next = nullptr;
        ftBodyElem* prev = nullptr;
    };

    ftBodyElem* bodies = nullptr;
    uint32 size = 0;

public:

    ftBody* create();
    void destroy(ftBody *body);

    uint32 getSize();

    class ftIter {
    private:
        ftBodyBuffer::ftBodyElem* curElem = nullptr;
        friend class ftBodyBuffer;
    };

    ftIter iterate();
    ftBody* start(ftIter* iter);
    ftBody* next(ftIter* iter);

};

#endif //FALTON_RIGIDBODY_H
