//
// Created by Kevin Yu on 2/21/16.
//


#include "falton/physics/Collision/ftCollisionSystem.h"
#include "falton/physics/Collision/BroadPhase/ftBroadPhase.h"


void ftCollisionSystem::init(ftBroadPhase *broadphase) {

    this->broadphase = broadphase;
    capacity = 64;
    shapes = new ftTransformShape[capacity];
    nShape = 0;
    nMinQueueSize = 0;

    this->broadphase->init(shapes);

    userData = new void*[capacity];
    curTimestamp = 0;
}

void ftCollisionSystem::shutdown() {

    broadphase->shutdown();
    delete[] shapes;
    delete[] userData;

}

ColHandle ftCollisionSystem::addShape(ftTransform transform, ftShape *shape, void *userData) {

    ColHandle freeHandle = nShape;
    if (handleQueue.getSize() > nMinQueueSize) {
        freeHandle = handleQueue.pop();
    }

    if (nShape == capacity) {
        ftTransformShape *oldShape = this->shapes;
        this->shapes = new ftTransformShape[capacity * 2];
        memcpy(this->shapes,oldShape,sizeof(capacity * sizeof(ftTransformShape)));
        capacity *= 2;
        delete oldShape;
    }

    this->shapes[freeHandle].transform = transform;
    this->shapes[freeHandle].shape = shape;
    this->userData[freeHandle] = userData;

    this->nShape++;

    broadphase->newShape(freeHandle);

    return freeHandle;

}

void ftCollisionSystem::removeShape(ColHandle handle) {

    broadphase->removeShape(handle);

    this->shapes[handle].shape = nullptr;

    handleQueue.push(handle);

    nMinQueueSize++;

}

void ftCollisionSystem::moveShape(ColHandle handle, ftTransform transform) {
    this->shapes[handle].transform = transform;
    broadphase->moveShape(handle);
}

void ftCollisionSystem::updateContacts(ftContactBuffer* contactBuffer, CollisionCallback callback) {
    ftChunkArray<ftBroadPhasePair> pairs(64);
    broadphase->findPairs(pairs);

    curTimestamp++;

    for (uint32 i=0;i<pairs.getSize();i++) {
        ftContact *contact = contactBuffer->find(pairs[i].handleA, pairs[i].handleB);
        if (contact == nullptr) {

            ftManifold manifold;
            ftManifoldComputer::Collide(shapes[pairs[i].handleA], shapes[pairs[i].handleB], manifold);
            if (manifold.numContact > 0) {
                contact = contactBuffer->create(pairs[i].handleA, pairs[i].handleB);
                contact->collisionState = BEGIN_COLLISION;
                contact->manifold = manifold;
                contact->userdataA = userData[contact->handleA];
                contact->userdataB = userData[contact->handleB];
                callback.beginContact(contact,callback.data);
                contact->timestamp = curTimestamp;
            }

        } else {
            contact->collisionState = IN_COLLISION;
            ftManifoldComputer::Collide(shapes[contact->handleA], shapes[contact->handleB], contact->manifold);
            if (contact->manifold.numContact > 0) {
                callback.updateContact(contact, callback.data);
                contact->timestamp = curTimestamp;
            }
        }
    }

    //destroy ending contact
    {
        ftChunkArray<ftContact*> endingContact(64);

        ftContactBuffer::ftIter iter = contactBuffer->iterate();

        for (ftContact* contact = contactBuffer->start(&iter); contact != nullptr;
             contact = contactBuffer->next(&iter)) {
            if (contact->timestamp != curTimestamp ||
                    this->shapes[contact->handleA].shape == nullptr ||
                    this->shapes[contact->handleB].shape == nullptr) {
                callback.endContact(contact, callback.data);
                endingContact.push(contact);
            }
        }

        for (uint32 i=0;i<endingContact.getSize();i++) {
            contactBuffer->destroy(endingContact[i]);
        }

    }

    nMinQueueSize = 0;
}
