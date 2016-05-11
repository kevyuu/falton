//
// Created by Kevin Yu on 2/21/16.
//

#ifndef FALTON_FTCOLLISIONSYSTEM_H
#define FALTON_FTCOLLISIONSYSTEM_H

#include <stdint.h>
#include <falton/math/math.h>
#include <falton/container/ftChunkArray.h>
#include <falton/container/ftIntQueue.h>
#include <falton/physics/collision/broadphase/ftBroadphaseSystem.h>

struct ftContact;

typedef void (*ContactCallbackFunc) (ftContact* contact, void *data);
typedef bool (*CollisionFilterFunc) (void* userdataA, void* userdataB);

typedef uint32 ftColHandle;

struct ftContact;
class ftShape;

class ftContactBuffer;

struct ftCollisionShape {
public:
    ftShape *shape;
    ftTransform transform;

    ftBroadphaseHandle broadHandle;
    void *userdata;
};

struct ftCollisionCallback {
    ContactCallbackFunc beginContact, updateContact, endContact;
    void *data;
};

class ftCollisionSystem {

public:

    void init(ftBroadphaseSystem *broadphase);
    void shutdown();

    ftColHandle addShape(ftTransform transform, ftShape *shape, void *userData);
    void removeShape(ftColHandle handle);
    void moveShape(ftColHandle handle, ftTransform transform);

    void updateContacts(ftContactBuffer *contactBuffer, CollisionFilterFunc filter, ftCollisionCallback callback);

private:

    ftChunkArray<ftCollisionShape> m_shapes;
    uint32 m_nShape;
    uint32 m_minQueueSize; //number of removed shape this current timestamp. Cannot reuse handle if shape remove in the same timestamp

    ftIntQueue m_handleQueue;

    uint8 m_curTimeStamp;

    ftBroadphaseSystem *m_broadphase;

};


#endif //FALTON_FTCOLLISIONSYSTEM_H
