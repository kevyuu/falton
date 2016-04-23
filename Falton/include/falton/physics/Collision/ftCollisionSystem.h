//
// Created by Kevin Yu on 2/21/16.
//

#ifndef FALTON_FTCOLLISIONSYSTEM_H
#define FALTON_FTCOLLISIONSYSTEM_H

#include "falton/math/math.h"
#include "falton/container/ftIntQueue.h"
#include "ftManifoldComputer.h"
#include "falton/physics/Collision/ftContact.h"

class ftBroadPhase;

typedef void (*ContactCallbackFunc) (ftContact* contact, void *data);

struct CollisionCallback {
    ContactCallbackFunc beginContact, updateContact, endContact;
    void *data;
};

class ftCollisionSystem {

public:

    ftBroadPhase *broadphase;

    void init(ftBroadPhase *broadphase);
    void shutdown();

    ColHandle addShape(ftTransform transform, ftShape *shape, void *userData);
    void removeShape(ColHandle handle);
    void moveShape(ColHandle handle, ftTransform transform);

    void updateContacts(ftContactBuffer *contactBuffer, CollisionCallback callback);

private:

    ftTransformShape* shapes;
    uint32 nShape;
    uint32 capacity;
    uint32 nMinQueueSize; //number of removed shape this current timestamp. Cannot reuse handle if shape remove in the same timestamp

    void **userData;

    ftIntQueue handleQueue;

    uint8 curTimestamp;

};


#endif //FALTON_FTCOLLISIONSYSTEM_H
