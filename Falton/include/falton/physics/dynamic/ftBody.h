//
// Created by Kevin Yu on 12/6/15.
//

#ifndef FALTON_RIGIDBODY_H
#define FALTON_RIGIDBODY_H

#include <falton/physics/joint/ftJoint.h>
#include "falton/math/math.h"

struct ftCollider;
struct ftBody;
struct ftContact;

enum ftBodyType {
    STATIC,
    KINEMATIC,
    DYNAMIC
};

enum ftActivationState {
    DISABLE_SLEEP,
    ACTIVE,
    SLEEP
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
    ftVector2 centerOfMass;

    ftBodyType bodyType;
    ftActivationState activationState = ACTIVE;

    ftVector2 forceAccum;
    ftVector2 velocity;

    real torqueAccum = 0;
    real angularVelocity = 0;

    real mass;
    real inverseMass;
    real moment;
    real inverseMoment;

    real sleepTimer = 0.0f;

    ftCollider* colliders = nullptr;

    ftContactEdge* contactList = nullptr;
    ftJointEdge* jointList = nullptr;

    int32 islandId = -1;

    void applyForce(ftVector2 force, ftVector2 localPos);
    void applyForceAtCenterOfMass(ftVector2 force);
    void applyTorque(real torque);

    void setMass(real mass);
    void setMoment(real moment);

    friend class ftPhysicsSystem;
    friend class ftIslandSystem;
    friend class ftConstraintSolver;

};

class ftBodyBuffer {

private:
    struct ftBodyElem {
        ftBody body;
        ftBodyElem* next = nullptr;
        ftBodyElem* prev = nullptr;
    };

    ftBodyElem* bodies;
    uint32 size;

public:

    ftBody* create();
    void destroy(ftBody *body);

    void insert(ftBody* body);
    void unlink(ftBody* body);

    uint32 getSize();

    class ftIter {
    private:
        ftBodyBuffer::ftBodyElem* curElem = nullptr;
        friend class ftBodyBuffer;
    };

    ftIter iterate();
    ftBody* start(ftIter* iter);
    ftBody* next(ftIter* iter);

    void init();
    void cleanup();

    template <typename T> void forEach(const T& f); // T will be a lambda type that takes ftBody* as argument */

};

/* T will be a lambda type that takes ftBody* as argument */
template <typename T>
void ftBodyBuffer::forEach(const T& f) {
    ftBodyElem* elem = bodies;
    while (elem != nullptr) {
        f((ftBody*)elem);
        elem = elem->next;
    }
}

#endif //FALTON_RIGIDBODY_H
