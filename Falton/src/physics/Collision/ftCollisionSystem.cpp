//
// Created by Kevin Yu on 2/21/16.
//


#include "falton/physics/Collision/ftCollisionSystem.h"
#include "falton/physics/Collision/BroadPhase/ftBroadPhase.h"

ftContactBuffer::ftContactBuffer() {
    contacts = new ftContact*[128];

    for (int i=0;i<128;++i) {
        contacts[i] = nullptr;
    }

    capacity = 128;
    nContact = 0;
}

ftContactBuffer::~ftContactBuffer() {
    delete contacts;
}

bool ftContactBuffer::exist(ColHandle handle1, ColHandle handle2) {

    ColHandle handleA,handleB;

    if (handle1 < handle2) {
        handleA = handle1;
        handleB = handle2;
    } else {
        handleA = handle2;
        handleB = handle1;
    }


    uint32 hashVal = hash(handleA, handleB, capacity);
    ftContact *nextContact = contacts[hashVal];
    while(nextContact) {
        if (nextContact->handleA == handleA && nextContact->handleB == handleB) return true;
        nextContact = nextContact->next;
    }
    return false;

}

ftContact* ftContactBuffer::newContact(ColHandle handle1, ColHandle handle2) {
    ftContact *contact = new ftContact();

    nContact++;

    if (nContact > capacity) {
        rehash(nContact * 2);
    }

    if (handle1 < handle2) {
        contact->handleA = handle1;
        contact->handleB = handle2;
    } else {
        contact->handleA = handle2;
        contact->handleB = handle1;
    }

    uint32 hashValue = hash(contact->handleA, contact->handleB, capacity);

    contact->next = contacts[hashValue];
    contacts[hashValue] = contact;

    return contact;
}

ftContact* ftContactBuffer::findContact(ColHandle handle1, ColHandle handle2) {

    ColHandle handleA,handleB;

    if (handle1 < handle2) {
        handleA = handle1;
        handleB = handle2;
    } else {
        handleA = handle2;
        handleB = handle1;
    }

    uint32 hashVal = hash(handleA, handleB, capacity);
    ftContact *nextContact = contacts[hashVal];
    while(nextContact) {
        if (nextContact->handleA == handleA && nextContact->handleB == handleB) return nextContact;
        nextContact = nextContact->next;
    }

    return nullptr;

}

uint32 ftContactBuffer::hash(ColHandle handleA, ColHandle handleB, uint32 capacity) {
    uint64 x = handleA + handleB;
    return ((x * (x + 1) /2) + handleB) % capacity;
}

uint64 ftContactBuffer::pairingFunction(ColHandle handleA, ColHandle handleB) {
    uint64 x = handleA + handleB;
    return ((x * (x + 1) /2) + handleB);
}

void ftContactBuffer::rehash(uint32 newCapacity) {

    ftContact **oldBuffer = contacts;
    contacts = new ftContact*[newCapacity];

    for (uint32 i=0;i<capacity;i++) {
        ftContact *nextContact = oldBuffer[i];
        while(nextContact) {
            uint32 hashVal = hash(nextContact->handleA, nextContact->handleB, newCapacity);
            ftContact *contact = nextContact;
            nextContact = nextContact->next;
            contact->next = contacts[hashVal];
            contacts[hashVal] = contact;
        }
    }

    delete oldBuffer;

    capacity = newCapacity;
}

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

void ftCollisionSystem::updateContacts(CollisionCallback callback) {
    ftChunkArray<ftBroadPhasePair> pairs(64);
    broadphase->findPairs(pairs);

    curTimestamp++;

    for (uint32 i=0;i<pairs.getSize();i++) {
        ftContact *contact = contactBuffer.findContact(pairs[i].handleA,pairs[i].handleB);
        if (contact == nullptr) {

            ftManifold manifold;
            ftManifoldComputer::Collide(shapes[pairs[i].handleA], shapes[pairs[i].handleB], manifold);
            if (manifold.numContact > 0) {
                contact = contactBuffer.newContact(pairs[i].handleA, pairs[i].handleB);
                contact->collisionState = BEGIN_COLLISION;
                contact->manifold = manifold;
                callback.beginContact(userData[contact->handleA],userData[contact->handleB],contact,callback.data);
                contact->timestamp = curTimestamp;
            }

        } else {
            contact->collisionState = IN_COLLISION;
            ftManifoldComputer::Collide(shapes[contact->handleA], shapes[contact->handleB], contact->manifold);
            if (contact->manifold.numContact > 0) {
                callback.updateContact(userData[contact->handleA], userData[contact->handleB], contact, callback.data);
                contact->timestamp = curTimestamp;
            }
        }
    }

    //delete ending contact
    for (uint32 i=0;i<contactBuffer.capacity;i++) {
        ftContact* contact = contactBuffer.contacts[i];
        ftContact* prevContact = nullptr;
        while (contact) {

            if (contact->timestamp != curTimestamp ||
                    this->shapes[contact->handleA].shape == nullptr ||
                    this->shapes[contact->handleB].shape == nullptr) {

                callback.endContact(userData[contact->handleA],userData[contact->handleB],contact,callback.data);
                if (prevContact == nullptr) {
                    contactBuffer.contacts[i] = contact->next;
                    delete contact;
                    contact = contactBuffer.contacts[i];
                } else {
                    prevContact->next = contact->next;
                    delete contact;
                    contact = prevContact->next;
                }

            } else {
                prevContact = contact;
                contact = contact->next;
            }
        }

    }

    nMinQueueSize = 0;
}

void ftCollisionSystem::iterateContacts(ContactCallbackFunc callback, void *data) {

    for (uint32 i=0;i<contactBuffer.capacity;i++) {
        ftContact* contact = contactBuffer.contacts[i];
        while (contact) {

            callback(userData[contact->handleA],userData[contact->handleB],contact,data);
            contact = contact->next;

        }
    }

}
