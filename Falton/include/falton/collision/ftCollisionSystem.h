//
// Created by Kevin Yu on 2/21/16.
//
#pragma once

#include <stdint.h>

#include "falton/math.h"
#include "falton/container/ftChunkArray.h"
#include "falton/container/ftIntQueue.h"
#include "falton/collision/broadphase/ftBroadphaseSystem.h"
#include "falton/collision/ftContact.h"
#include "falton/container/ftBitSet.h"
#include "falton/container/ftVectorArray.h"


struct ftContact;
struct ftShape;

typedef void (*ftContactCallbackFunc)(ftContact *contact, void *data);
typedef bool (*ftCollisionFilterFunc)(void *userdataA, void *userdataB);
typedef uint32 ftColHandle;


struct ftCollisionShape
{
  public:
    ftShape *shape;
    ftTransform transform;

    ftBroadphaseHandle broadHandle;
    union {
        void *userdata;
        ftColHandle nextFree;
    };
};

struct ftCollisionCallback
{
    ftContactCallbackFunc beginContact, updateContact, endContact;
    void *data;
};

/* Role : System */
class ftCollisionSystem
{

  public:
    void init(ftBroadphaseSystem *broadphase);
    void shutdown();

    ftColHandle addShape(ftTransform transform, ftShape *shape, void *userData);
    void removeShape(ftColHandle handle);
    void moveShape(ftColHandle handle, ftTransform transform);

    void updateContacts(ftCollisionFilterFunc filter, ftCollisionCallback callback);
    void updateOneAtATime(ftCollisionFilterFunc filter, ftCollisionCallback callback);
    void updateAllAtOnce(ftCollisionFilterFunc filter, ftCollisionCallback callback);
	void regionQuery(ftAABB region, ftVectorArray<void*>* results);

    void destroyContact(ftContact *contact);

    void setSleepRatio(real ratio)
    {
        m_sleepRatio = ratio;
    }

    template <typename T>
    void forEachContact(T func);

  private:
    ftChunkArray<ftCollisionShape> m_shapes;
    ftColHandle m_freeShapes;
    uint32 m_nShape;
    uint32 m_nFreeShape;

    // Number of removed shape this current timestamp.
    // Cannot reuse handle if shape removed in the same timestamp
    uint32 m_minQueueSize;

    real m_sleepRatio;
    uint8 m_curTimeStamp;

    ftBroadphaseSystem *m_broadphase;
    ftContactBuffer m_contactBuffer;

    ftBitSet m_moveMasks;
    ftChunkArray<ftColHandle> m_movedShapes;

    void updateContact(ftContact *contact,
                       ftColHandle handleA,
                       ftColHandle handleB,
                       ftCollisionCallback callback);
    void updateContact(ftContact *contact,
                       ftColHandle handleA,
                       ftColHandle handleB);
    void destroyEndingContacts(ftCollisionCallback callback);
};

inline void ftCollisionSystem::destroyContact(ftContact *contact)
{
    m_contactBuffer.destroy(contact);
}

template <typename T>
inline void ftCollisionSystem::forEachContact(T func)
{
    m_contactBuffer.forEach(func);
}