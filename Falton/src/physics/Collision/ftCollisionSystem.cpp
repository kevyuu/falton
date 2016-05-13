//
// Created by Kevin Yu on 2/21/16.
//


#include "falton/physics/collision/ftCollisionSystem.h"
#include <falton/physics/collision/ftContact.h>

#include <iostream>
#include <falton/physics/collision/ftManifoldComputer.h>

using namespace std;

void ftCollisionSystem::init(ftBroadphaseSystem *broadphase) {

    this->m_broadphase = broadphase;
    m_curTimeStamp = 0;
    m_nShape = 0;
    m_minQueueSize = 0;

    m_shapes.init(64);
    broadphase->init();
    m_handleQueue.init();

}

void ftCollisionSystem::shutdown() {

    m_broadphase->shutdown();
    m_shapes.cleanup();
    m_handleQueue.cleanup();

}

ftColHandle ftCollisionSystem::addShape(ftTransform transform, ftShape* shape, void* userData) {
    ftColHandle freeHandle = this->m_nShape;
    if (m_handleQueue.getSize() > m_minQueueSize) {
        freeHandle = m_handleQueue.pop();
    } else {
        freeHandle = m_shapes.add();
    }

    ftCollisionShape* colShape = &(m_shapes[freeHandle]);
    colShape->transform = transform;
    colShape->shape = shape;
    colShape->userdata = userData;

    this->m_nShape++;

    void* broadphaseProxy = (void*) freeHandle;
    m_shapes[freeHandle].broadHandle = m_broadphase->addShape(colShape, broadphaseProxy);

    return freeHandle;

}

void ftCollisionSystem::removeShape(ftColHandle handle) {

    ftBroadphaseHandle broadHandle = m_shapes[handle].broadHandle;
    m_broadphase->removeShape(broadHandle);

    m_shapes[handle].shape = nullptr;

    m_handleQueue.push(handle);
    m_minQueueSize++;
    this->m_nShape--;

}

void ftCollisionSystem::moveShape(ftColHandle handle, ftTransform transform) {
    m_shapes[handle].transform = transform;

    ftBroadphaseHandle broadHandle = m_shapes[handle].broadHandle;

    m_broadphase->moveShape(broadHandle, m_shapes[handle]);
}

void ftCollisionSystem::updateContacts(ftContactBuffer* contactBuffer, ftCollisionFilterFunc filter, ftCollisionCallback callback) {
    ftChunkArray<ftBroadPhasePair> pairs;
    pairs.init(64);

    m_broadphase->findPairs(&pairs);

    m_curTimeStamp++;

    for (uint32 i=0;i<pairs.getSize();i++) {
        const ftColHandle handleA = (ftColHandle) pairs[i].userdataA;
        const ftColHandle handleB = (ftColHandle) pairs[i].userdataB;
        const ftCollisionShape* colShapeA = &m_shapes[handleA];
        const ftCollisionShape* colShapeB = &m_shapes[handleB];

        ftContact *contact = contactBuffer->find(handleA, handleB);

        if (filter(colShapeA->userdata, colShapeB->userdata)) {
            if (contact == nullptr) {

                ftManifold manifold;
                ftManifoldComputer::Collide(*colShapeA, *colShapeB, &manifold);
                if (manifold.numContact > 0) {
                    contact = contactBuffer->create(handleA, handleB);
                    contact->collisionState = BEGIN_COLLISION;
                    contact->manifold = manifold;
                    contact->userdataA = colShapeA->userdata;
                    contact->userdataB = colShapeB->userdata;
                    callback.beginContact(contact, callback.data);
                    contact->timestamp = m_curTimeStamp;
                }

            } else {
                contact->collisionState = IN_COLLISION;
                ftManifoldComputer::Collide(*colShapeA, *colShapeB, &(contact->manifold));
                if (contact->manifold.numContact > 0) {
                    callback.updateContact(contact, callback.data);
                    contact->timestamp = m_curTimeStamp;
                }
            }
        }

    }

    pairs.cleanup();

    //destroy ending contact
    {
        ftChunkArray<ftContact*> endingContact;
        endingContact.init(64);

        ftContactBuffer::ftIter iter = contactBuffer->iterate();

        for (ftContact* contact = contactBuffer->start(&iter); contact != nullptr;
             contact = contactBuffer->next(&iter)) {
            if (contact->timestamp != m_curTimeStamp ||
                        m_shapes[contact->handleA].shape == nullptr ||
                        m_shapes[contact->handleB].shape == nullptr) {
                callback.endContact(contact, callback.data);
                endingContact.push(contact);
            }
        }

        for (uint32 i=0;i<endingContact.getSize();i++) {
            contactBuffer->destroy(endingContact[i]);
        }

    }

    m_minQueueSize = 0;
}
