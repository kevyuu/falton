//
// Created by Kevin Yu on 2/21/16.
//


#include "falton/physics/collision/ftCollisionSystem.h"
#include <falton/physics/collision/ftContact.h>

#include <iostream>
#include <falton/physics/collision/ftManifoldComputer.h>
#include <ftBenchmark.h>

using namespace std;

void ftCollisionSystem::init(ftBroadphaseSystem *broadphase) {

    this->m_broadphase = broadphase;
    m_curTimeStamp = 0;
    m_nShape = 0;
    m_minQueueSize = 0;

    m_shapes.init(64);
    broadphase->init();
    m_handleQueue.init();
    m_contactBuffer.init();

    m_moveMasks.init(64);
}

void ftCollisionSystem::shutdown() {

    m_broadphase->shutdown();
    m_shapes.cleanup();
    m_handleQueue.cleanup();
    m_contactBuffer.cleanup();
    m_moveMasks.cleanup();
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

    int32 maskCapacity = m_moveMasks.getCapacity();
    if (freeHandle >= maskCapacity) {
        m_moveMasks.resize(2 * maskCapacity);
    }
    m_moveMasks.on(freeHandle);

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

    m_moveMasks.on(handle);
}

void ftCollisionSystem::updateContacts(ftCollisionFilterFunc filter,
                                       ftCollisionCallback callback) {
    ftChunkArray<ftBroadPhasePair> pairs;
    pairs.init(64);

    static int findPairIdx = -1;
    findPairIdx = ftBenchmark::Begin("ftBroadphase::findPairs", findPairIdx);
    m_broadphase->findPairs(&pairs);
    ftBenchmark::End();

    m_curTimeStamp++;

    static int updateColliderIdx = -1;
    updateColliderIdx = ftBenchmark::Begin("Update Collide",updateColliderIdx);
    for (uint32 i=0;i<pairs.getSize();i++) {
        ftColHandle handleA = (ftColHandle) pairs[i].userdataA;
        ftColHandle handleB = (ftColHandle) pairs[i].userdataB;

        if (handleA > handleB) {
            ftColHandle tmp = handleA;
            handleA = handleB;
            handleB = tmp;
        }

        const ftCollisionShape* colShapeA = &m_shapes[handleA];
        const ftCollisionShape* colShapeB = &m_shapes[handleB];

        void* userdataA = m_shapes[handleA].userdata;
        void* userdataB = m_shapes[handleB].userdata;

        if (!filter(userdataA, userdataB)) continue;

        ftContact *contact = m_contactBuffer.find(handleA, handleB);

        //if both shapes are not moving, manifold should be the same as previous frame
        if (!m_moveMasks.test(handleA) && !m_moveMasks.test(handleB)) {
            if (contact!=nullptr)contact->timestamp = m_curTimeStamp;
            continue;
        }

        static int lalaIdx = -1;
        lalaIdx = ftBenchmark::Begin("lala",lalaIdx);
        if (contact == nullptr) {
            ftManifold manifold;
            ftManifoldComputer::Collide(*colShapeA, *colShapeB, &manifold);
            if (manifold.numContact > 0) {
                contact = m_contactBuffer.create(handleA, handleB);
                contact->collisionState = BEGIN_COLLISION;
                contact->manifold = manifold;
                contact->userdataA = colShapeA->userdata;
                contact->userdataB = colShapeB->userdata;
                callback.beginContact(contact, callback.data);
                contact->timestamp = m_curTimeStamp;
            }

        } else if (contact->timestamp != m_curTimeStamp) {
            contact->collisionState = IN_COLLISION;
            ftManifold oldManifold = contact->manifold;
            ftManifoldComputer::Collide(*colShapeA, *colShapeB, &(contact->manifold));
            if (contact->manifold.numContact > 0) {
                if (oldManifold.numContact != contact->manifold.numContact) {
                    contact->manifold.contactPoints[0].tIAcc = 0;
                    contact->manifold.contactPoints[0].nIAcc = 0;
                    contact->manifold.contactPoints[1].tIAcc = 0;
                    contact->manifold.contactPoints[1].nIAcc = 0;
                }
                callback.updateContact(contact, callback.data);
                contact->timestamp = m_curTimeStamp;
            }
        }
        ftBenchmark::End();

    }

    ftBenchmark::End();

    pairs.cleanup();

    //destroy ending contact
    {
        ftChunkArray<ftContact*> endingContact;
        endingContact.init(64);

        for (auto iter = m_contactBuffer.iterate(); iter.contact != nullptr; m_contactBuffer.next(&iter)) {
            if (iter.contact->timestamp != m_curTimeStamp ||
                    m_shapes[iter.handleA].shape == nullptr ||
                    m_shapes[iter.handleB].shape == nullptr) {
                callback.endContact(iter.contact, callback.data);
                endingContact.push(iter.contact);
            }
        }

        for (uint32 i=0;i<endingContact.getSize();i++) {
            m_contactBuffer.destroy(endingContact[i]);
        }

    }

    m_minQueueSize = 0;
    m_moveMasks.clear();
}
