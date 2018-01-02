//
// Created by Kevin Yu on 2/21/16.
//

#include <falton/collision/ftCollisionSystem.h>
#include <falton/collision/ftManifoldComputer.h>
#include <functional>
#include <iostream>

void ftCollisionSystem::init(ftBroadphaseSystem *broadphase)
{

    this->m_broadphase = broadphase;
    m_curTimeStamp = 0;
    m_nShape = 0;
    m_minQueueSize = 0;
    m_shapes.init(64);
    broadphase->init();
    m_contactBuffer.init();
    m_moveMasks.init(64);
    m_movedShapes.init(64);
    m_freeShapes = nulluint;
    m_nFreeShape = 0;
}

void ftCollisionSystem::shutdown()
{

    m_broadphase->shutdown();
    m_shapes.cleanup();
    m_contactBuffer.cleanup();
    m_moveMasks.cleanup();
    m_movedShapes.cleanup();
}

ftColHandle ftCollisionSystem::addShape(ftTransform transform,
                                        ftShape *shape,
                                        void *userData)
{
    ftColHandle freeHandle;
    if (m_nFreeShape > m_minQueueSize)
    {
        freeHandle = m_freeShapes;
        m_freeShapes = m_shapes[freeHandle].nextFree;
        --m_nFreeShape;
    }
    else
    {
        freeHandle = m_shapes.push();
    }

    ftCollisionShape *colShape = &(m_shapes[freeHandle]);
    colShape->transform = transform;
    colShape->shape = shape;
    colShape->userdata = userData;

    m_nShape++;

    void *broadphaseProxy = (void *)freeHandle;
    m_shapes[freeHandle].broadHandle = m_broadphase->addShape(shape, transform, broadphaseProxy);

    uint32 maskCapacity = m_moveMasks.getCapacity();
    while (freeHandle >= maskCapacity)
    {
        m_moveMasks.resize(2 * maskCapacity);
        maskCapacity = m_moveMasks.getCapacity();
    }
    m_moveMasks.on(freeHandle);
    m_movedShapes.push(freeHandle);

    return freeHandle;
}

void ftCollisionSystem::removeShape(ftColHandle handle)
{

    ftBroadphaseHandle broadHandle = m_shapes[handle].broadHandle;
    m_broadphase->removeShape(broadHandle);

    m_shapes[handle].shape = nullptr;

    m_shapes[handle].nextFree = m_freeShapes;
    m_freeShapes = handle;
    ++m_nFreeShape;

    m_minQueueSize++;
    m_moveMasks.off(handle);
    m_nShape--;
}

void ftCollisionSystem::moveShape(ftColHandle handle, ftTransform transform)
{
    m_shapes[handle].transform = transform;

    ftBroadphaseHandle broadHandle = m_shapes[handle].broadHandle;

    m_broadphase->moveShape(broadHandle, m_shapes[handle].shape, m_shapes[handle].transform);

    m_moveMasks.on(handle);
    m_movedShapes.push(handle);
}

void ftCollisionSystem::updateContacts(ftCollisionFilterFunc filter,
                                       ftCollisionCallback callback)
{
    if (m_sleepRatio * m_nShape > m_movedShapes.getSize())
    {
        updateOneAtATime(filter, callback);
    }
    else
    {
        updateAllAtOnce(filter, callback);
    }
}

void ftCollisionSystem::updateAllAtOnce(ftCollisionFilterFunc filter,
                                        ftCollisionCallback callback)
{

    ++m_curTimeStamp;

    ftChunkArray<ftBroadPhasePair> pairs;
    pairs.init(128);

    m_broadphase->findPairs(&pairs);

    for (uint32 i = 0; i < pairs.getSize(); i++)
    {
        ftColHandle handleA = *(ftColHandle *)(&pairs[i].userdataA);
        ftColHandle handleB = *(ftColHandle *)(&pairs[i].userdataB);
        if (handleA > handleB)
        {
            ftColHandle tmp = handleA;
            handleA = handleB;
            handleB = tmp;
        }
        void *userdataA = m_shapes[handleA].userdata;
        void *userdataB = m_shapes[handleB].userdata;
        if (!filter(userdataA, userdataB))
            continue;

        ftContact *contact = m_contactBuffer.find(handleA, handleB);
        bool isMoving = this->m_moveMasks.test(handleA) || this->m_moveMasks.test(handleB);
        if (!isMoving)
        {
            if (contact != nullptr)
                contact->timestamp = m_curTimeStamp;
            continue;
        }

        updateContact(contact, handleA, handleB, callback);
    }

    pairs.cleanup();

    destroyEndingContacts(callback);

    m_minQueueSize = 0;
    m_moveMasks.clear();
    m_movedShapes.removeAll();
}

void ftCollisionSystem::updateOneAtATime(ftCollisionFilterFunc filter,
                                         ftCollisionCallback callback)
{
    ++m_curTimeStamp;

    const auto updateIdleContactTimestamp = [this, filter](ftColHandle handleA,
                                                           ftColHandle handleB,
                                                           ftContact *contact) {
        bool isMoving = this->m_moveMasks.test(handleA) || this->m_moveMasks.test(handleB);
        bool isExist = this->m_shapes[handleA].shape != nullptr && this->m_shapes[handleB].shape != nullptr;
        void *userdataA = this->m_shapes[handleA].userdata;
        void *userdataB = this->m_shapes[handleB].userdata;
        if (isExist && !isMoving && filter(userdataA, userdataB))
        {
            contact->timestamp = m_curTimeStamp;
        }
    };

    m_contactBuffer.forEach(std::cref(updateIdleContactTimestamp));

    ftChunkArray<const void *> results;
    results.init(128);
    for (uint32 i = 0; i < m_movedShapes.getSize(); ++i)
    {
        ftColHandle handle = m_movedShapes[i];
        ftAABB region = m_shapes[handle].shape->constructAABB(m_shapes[handle].transform);
        m_broadphase->regionQuery(region, &results);

        for (uint32 j = 0; j < results.getSize(); ++j)
        {
            ftColHandle resultHandle = *(ftColHandle *)&results[j];
            if (resultHandle == handle)
                continue;

            uint32 handleA = handle;
            uint32 handleB = resultHandle;
            if (handleA > handleB)
            {
                handleA = resultHandle;
                handleB = handle;
            }

            void *userdataA = m_shapes[handleA].userdata;
            void *userdataB = m_shapes[handleB].userdata;
            if (!filter(userdataA, userdataB))
                continue;
            ftContact *contact = m_contactBuffer.find(handleA, handleB);

            updateContact(contact, handleA, handleB, callback);
        }

        results.removeAll();
    }

    results.cleanup();

    destroyEndingContacts(callback);

    m_minQueueSize = 0;
    m_moveMasks.clear();
    m_movedShapes.removeAll();
}


void ftCollisionSystem::regionQuery(ftAABB region, ftVectorArray<void*>* userdataResults) {
	ftChunkArray<const void *> results;
	results.init(128);
    m_broadphase->regionQuery(region, &results);
	for (int j = 0; j < results.getSize(); ++j) {
		ftColHandle handle = (ftColHandle)results[j];
		void* userdata = m_shapes[handle].userdata;
		userdataResults->push(userdata);
	}
	results.cleanup();
}

void ftCollisionSystem::destroyEndingContacts(ftCollisionCallback callback)
{
    const auto processEndContacts = [&callback, this](ftColHandle handleA,
                                                      ftColHandle handleB,
                                                      ftContact *contact) {
        if (contact->timestamp != this->m_curTimeStamp)
        {
            callback.endContact(contact, callback.data);
        }
    };

    m_contactBuffer.forEach(std::cref(processEndContacts));

    const auto isEnding = [this](ftColHandle handleA,
                                 ftColHandle handleB,
                                 ftContact *contact) {
        return contact->timestamp != this->m_curTimeStamp;
    };

    m_contactBuffer.removeIf(std::cref(isEnding));
}

void ftCollisionSystem::updateContact(ftContact *contact,
                                      ftColHandle handleA,
                                      ftColHandle handleB,
                                      ftCollisionCallback callback)
{
    const ftCollisionShape *colShapeA = &m_shapes[handleA];
    const ftCollisionShape *colShapeB = &m_shapes[handleB];
    void *userdataA = m_shapes[handleA].userdata;
    void *userdataB = m_shapes[handleB].userdata;

    if (contact == nullptr)
    {
        ftManifold manifold;
        ftManifoldComputer::Collide(*colShapeA, *colShapeB, &manifold);
        if (manifold.numContact > 0)
        {
            contact = m_contactBuffer.create(handleA, handleB);
            contact->collisionState = BEGIN_COLLISION;
            contact->manifold = manifold;
            contact->userdataA = userdataA;
            contact->userdataB = userdataB;
            callback.beginContact(contact, callback.data);
            contact->timestamp = m_curTimeStamp;
        }
    }
    else if (contact->timestamp != m_curTimeStamp)
    {
        contact->collisionState = IN_COLLISION;
        ftManifold oldManifold = contact->manifold;

        ftManifoldComputer::Collide(*colShapeA, *colShapeB, &(contact->manifold));
        if (contact->manifold.numContact > 0)
        {
            if (oldManifold.numContact != contact->manifold.numContact)
            {
                contact->manifold.contactPoints[0].tIAcc = 0;
                contact->manifold.contactPoints[0].nIAcc = 0;
                contact->manifold.contactPoints[1].tIAcc = 0;
                contact->manifold.contactPoints[1].nIAcc = 0;
            }
            callback.updateContact(contact, callback.data);
            contact->timestamp = m_curTimeStamp;
        }
    }
}
