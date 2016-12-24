//
// Created by Kevin Yu on 12/6/15.
//

#ifndef FALTON_RIGIDBODY_H
#define FALTON_RIGIDBODY_H

#include <falton/dynamic/ftJoint.h>
#include <falton/math.h>

struct ftCollider;
struct ftBody;
struct ftContact;

enum ftBodyType {
    STATIC,
    KINEMATIC,
    DYNAMIC,
    COUNT_BODY_TYPE
};

enum ftActivationState {
    ACTIVE,
    SLEEP,
};

struct ftContactEdge {
    ftBody* other = nullptr;
    ftContact* contact = nullptr;
    ftContactEdge* prev = nullptr;
    ftContactEdge* next = nullptr;
};

class ftBody {

private:

    ftBodyType bodyType;
    ftActivationState activationState = ACTIVE;

    ftVector2 forceAccum;

    real torqueAccum = 0;
    
    real mass;
    real inverseMass;
    real moment;
    real inverseMoment;

    real sleepTimer = 0.0f;

    ftCollider* colliders = nullptr;

    ftContactEdge* contactList = nullptr;
    ftJointEdge* jointList = nullptr;

    int32 islandId = -1;

    friend class ftPhysicsSystem;
    friend class ftIslandSystem;
    friend class ftConstraintSolver;
    friend class ftJointSolver;
    
    friend class ftDistanceJoint;
    friend class ftDynamoJoint;
    friend class ftHingeJoint;
    friend class ftPistonJoint;
    friend class ftSpringJoint;
    
public:

    void* userdata;

    ftTransform transform;
    ftVector2 centerOfMass;
    ftVector2 velocity;
    real angularVelocity = 0;

    void applyForce(ftVector2 force, ftVector2 localPos);
    void applyForceAtCenterOfMass(ftVector2 force);  
    void applyTorque(real torque);

    void setMass(real mass);
    void setMoment(real moment);

    real getMass();
    real getMoment();

    void addContactEdge(ftContactEdge* contactEdge);
    void addJointEdge(ftJointEdge* jointEdge);
    void addCollider(ftCollider* collider);

    template <typename T>
    void forEachCollider(T func);

    template <typename T>
    void forEachContact(T func);

    template <typename T>
    void forEachJoint(T func);

};

template <typename T>
void ftBody::forEachCollider(T func) {
    for (ftCollider* collider = colliders; collider != nullptr; collider = collider->next) {
        func(collider);
    }
}

template <typename T>
void ftBody::forEachContact(T func) {
    for (ftContactEdge* contactEdge = contactList; contactEdge != nullptr; contactEdge = contactEdge->next) {
        func(contactEdge->contact);
    }
}

template <typename T>
void ftBody::forEachJoint(T func) {
    for (ftJointEdge* jointEdge = jointList; jointEdge != nullptr; jointEdge = jointEdge->next) {
        func(jointEdge->joint);
    }
}

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

    /* template T {
     *  operator(ftBody* body);
     * }*/
    template <typename T> void forEach(const T& f);

};

template <typename T>
void ftBodyBuffer::forEach(const T& f) {
    ftBodyElem* elem = bodies;
    while (elem != nullptr) {
        f((ftBody*)elem);
        elem = elem->next;
    }
}

#endif //FALTON_RIGIDBODY_H
