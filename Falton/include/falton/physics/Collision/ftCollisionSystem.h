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
#include <falton/physics/collision/ftContact.h>
#include <falton/container/ftBitSet.h>

struct ftContact;

typedef void (*ftContactCallbackFunc) (ftContact* contact, void *data);
typedef bool (*ftCollisionFilterFunc) (void* userdataA, void* userdataB);

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
    ftContactCallbackFunc beginContact, updateContact, endContact;
    void *data;
};

class ftCollisionSystem {

public:

    void init(ftBroadphaseSystem *broadphase);
    void shutdown();

    ftColHandle addShape(ftTransform transform, ftShape *shape, void *userData);
    void removeShape(ftColHandle handle);
    void moveShape(ftColHandle handle, ftTransform transform);

    void updateContacts(ftCollisionFilterFunc filter, ftCollisionCallback callback);

    void destroyContact(ftContact* contact);

    template <typename T>
    void forEachContact(T func);

private:

    ftChunkArray<ftCollisionShape> m_shapes;
    uint32 m_nShape;
    uint32 m_minQueueSize; //number of removed shape this current timestamp. Cannot reuse handle if shape remove in the same timestamp

    ftIntQueue m_handleQueue;

    uint8 m_curTimeStamp;

    ftBroadphaseSystem *m_broadphase;
    ftContactBuffer m_contactBuffer;

    ftBitSet m_moveMasks;

};

inline void ftCollisionSystem::destroyContact(ftContact *contact) {
    m_contactBuffer.destroy(contact);
}

template <typename T>
inline void ftCollisionSystem::forEachContact(T func) {
    m_contactBuffer.forEach(func);
}


#endif //FALTON_FTCOLLISIONSYSTEM_H
