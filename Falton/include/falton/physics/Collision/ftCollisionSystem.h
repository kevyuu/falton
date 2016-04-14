//
// Created by Kevin Yu on 2/21/16.
//

#ifndef FALTON_FTCOLLISIONSYSTEM_H
#define FALTON_FTCOLLISIONSYSTEM_H

#include "falton/math/math.h"
#include "falton/container/ftIntQueue.h"
#include "ftManifoldComputer.h"

typedef uint32 ColHandle;

typedef enum ftCollisionState {

    BEGIN_COLLISION,
    IN_COLLISION

} ftCollisionState;

class ftShape;
struct ftTransformShape;

struct ftContact {

    ColHandle handleA, handleB; //handle A alwalys < handle B
    ftManifold manifold;
    ftContact *next;

    ftCollisionState collisionState;

    uint8 timestamp;

    void* constraint;
};

typedef void (*ContactCallbackFunc) (void* userDataA, void* userDataB, ftContact* contact, void *data);

struct CollisionCallback {
    ContactCallbackFunc beginContact, updateContact, endContact;
    void *data;
};

//TODO contact iter and delete contact method
class ftContactBuffer {

public:

    ftContact **contacts;
    uint32 capacity;
    uint32 nContact;

    ftContactBuffer();
    ~ftContactBuffer();

    bool exist(ColHandle handle1, ColHandle handle2);

    ftContact* newContact(ColHandle handle1, ColHandle handle2);

    ftContact* findContact(ColHandle handle1, ColHandle handle2);

    uint32 hash(ColHandle handleA, ColHandle handleB, uint32 capacity);

    uint64 pairingFunction(ColHandle handleA, ColHandle handleB);

    void rehash(uint32 newCapacity);
};

class ftBroadPhase;

class ftCollisionSystem {

public:

    ftBroadPhase *broadphase;

    void init(ftBroadPhase *broadphase);
    void shutdown();

    ColHandle addShape(ftTransform transform, ftShape *shape, void *userData);
    void removeShape(ColHandle handle);
    void moveShape(ColHandle handle, ftTransform transform);

    void updateContacts(CollisionCallback callback);
    void iterateContacts(ContactCallbackFunc callback, void *data);

private:

    ftTransformShape* shapes;
    uint32 nShape;
    uint32 capacity;
    uint32 nMinQueueSize; //number of removed shape this current timestamp. Cannot reuse handle if shape remove in the same timestamp

    void **userData;

    ftIntQueue handleQueue;

    ftContactBuffer contactBuffer;

    uint8 curTimestamp;

};


#endif //FALTON_FTCOLLISIONSYSTEM_H
