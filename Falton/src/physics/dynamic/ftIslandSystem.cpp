//
// Created by Kevin Yu on 4/20/16.
//

#include "falton/physics/dynamic/ftIslandSystem.h"
#include "falton/physics/dynamic/ftCollider.h"
#include "falton/physics/dynamic/ftBody.h"
#include "falton/physics/dynamic/ftIsland.h"
#include "falton/physics/collision/ftContact.h"


void ftIslandSystem::init(ftBodyBuffers bodyBuffers) {
    this->m_buffers = bodyBuffers;
    m_islandContacts.init(64);
}

void ftIslandSystem::shutdown() {
    m_islandContacts.cleanup();
}

void ftIslandSystem::addContact(ftContact *contact) {

    ftCollider* colliderA = (ftCollider*) contact->userdataA;
    ftCollider* colliderB = (ftCollider*) contact->userdataB;

    ftBody* bodyA = colliderA->body;
    ftBody* bodyB = colliderB->body;

    ftContactEdge* contactEdgeA = createContactEdge(bodyA, bodyB, contact);
    ftContactEdge* contactEdgeB = createContactEdge(bodyB, bodyA, contact);

    int index = m_islandContacts.add();
    contact->islandIndex = index;
    m_islandContacts[index].contact = contact;
    m_islandContacts[index].contactEdgeA = contactEdgeA;
    m_islandContacts[index].contactEdgeB = contactEdgeB;


}

void ftIslandSystem::removeContact(ftContact *contact) {

    ftIslandContact islandContact = m_islandContacts[contact->islandIndex];
    destroyContactEdge(islandContact.contactEdgeA);
    destroyContactEdge(islandContact.contactEdgeB);

    uint32 lastIndex = m_islandContacts.getSize() - 1;
    ftContact* lastContact = m_islandContacts[lastIndex].contact;
    lastContact->islandIndex = contact->islandIndex;
    m_islandContacts[contact->islandIndex] = m_islandContacts[lastIndex];
    m_islandContacts.remove();

}


ftContactEdge* ftIslandSystem::createContactEdge(ftBody *body1, ftBody *body2, ftContact *contact) {

    ftContactEdge* contactEdge = new ftContactEdge;
    contactEdge->other = body2;

    contactEdge->next = body1->contactList;
    if (body1->contactList != nullptr) body1->contactList->prev = contactEdge;
    contactEdge->prev = nullptr;
    contactEdge->contact = contact;

    body1->contactList = contactEdge;

    return contactEdge;
}

void ftIslandSystem::destroyContactEdge(ftContactEdge *contactEdge) {

    ftContactEdge* prev = contactEdge->prev;
    ftContactEdge* next = contactEdge->next;

    ftContact* contact = contactEdge->contact;
    ftCollider* colliderA = (ftCollider*) contact->userdataA;
    ftCollider* colliderB = (ftCollider*) contact->userdataB;

    ftBody* bodyA = colliderA->body;
    ftBody* bodyB = colliderB->body;

    ftBody* body;
    if (bodyA == contactEdge->other) body = bodyB;
    else body = bodyA;

    if (prev!=nullptr) prev->next = next;
    else body->contactList = next;
    if (next!=nullptr) next->prev = prev;

    delete contactEdge;
}

ftIslandSystem::ftIter ftIslandSystem::iterate() {
    resetIslandID(m_buffers.dynamicBuffer);
    resetContactFlag();
    ftIter iter;
    iter.iter = m_buffers.dynamicBuffer->iterate();
    return iter;
}

ftIsland* ftIslandSystem::start(ftIter *iter) {

    int nBody  = m_buffers.dynamicBuffer->getSize() + m_buffers.staticBuffer->getSize() + m_buffers.kinematicBuffer->getSize();

    //build island with dfs
    ftBodyBuffer *bodyBuffer = m_buffers.dynamicBuffer;
    for (ftBody *body = bodyBuffer->start(&(iter->iter)); body != nullptr; body = bodyBuffer->next(&(iter->iter))) {
        if (body->islandId == -1) {

            resetIslandID(m_buffers.staticBuffer);

            ftBody **dfsStack = new ftBody *[nBody];
            int stackSize = 0;

            ftIsland *island = new ftIsland;
            island->bodies.init(64);
            island->contacts.init(64);

            dfsStack[0] = body;
            ++stackSize;

            while (stackSize > 0) {
                ftBody *topBody = dfsStack[stackSize - 1];
                --stackSize;

                if (topBody->islandId != -1) continue;

                int index = island->bodies.add();
                island->bodies[index] = topBody;
                topBody->islandId = index;

                for (ftContactEdge *contactEdge = topBody->contactList;
                     contactEdge != nullptr; contactEdge = contactEdge->next) {

                    ftContact* contact = contactEdge->contact;

                    ftIslandContact* islandContact = &(m_islandContacts[contact->islandIndex]);
                    if (islandContact->dfsFlag) {
                        continue;
                    }

                    island->contacts.push(contactEdge->contact);
                    islandContact->dfsFlag = true;

                    dfsStack[stackSize] = contactEdge->other;
                    ++stackSize;
                }
            }

            delete[] dfsStack;
            iter->prevIsland = island;
            return island;
        }

    }

    return nullptr;

}

ftIsland* ftIslandSystem::next(ftIter *iter) {

    iter->prevIsland->bodies.cleanup();
    iter->prevIsland->contacts.cleanup();
    delete iter->prevIsland;

    int nBody  = m_buffers.dynamicBuffer->getSize() + m_buffers.staticBuffer->getSize() + m_buffers.kinematicBuffer->getSize();
    ftBodyBuffer *bodyBuffer = m_buffers.dynamicBuffer;
    for (ftBody *body = bodyBuffer->next(&(iter->iter)); body != nullptr; body = bodyBuffer->next(&(iter->iter))) {
        if (body->islandId == -1) {

            resetIslandID(m_buffers.staticBuffer);
            resetIslandID(m_buffers.kinematicBuffer);

            ftBody **dfsStack = new ftBody *[nBody];
            int stackSize = 0;

            ftIsland *island = new ftIsland;

            dfsStack[0] = body;
            ++stackSize;

            while (stackSize > 0) {
                ftBody *topBody = dfsStack[stackSize - 1];
                --stackSize;

                if (topBody->islandId != -1) continue;

                int index = island->bodies.add();
                island->bodies[index] = topBody;
                topBody->islandId = index;

                for (ftContactEdge *contactEdge = topBody->contactList;
                     contactEdge != nullptr; contactEdge = contactEdge->next) {

                    ftContact* contact = contactEdge->contact;
                    ftIslandContact* islandContact = &(m_islandContacts[contact->islandIndex]);
                    if (islandContact->dfsFlag) {
                        continue;
                    }

                    island->contacts.push(contactEdge->contact);
                    islandContact->dfsFlag = true;

                    dfsStack[stackSize] = contactEdge->other;
                    ++stackSize;
                }
            }

            delete[] dfsStack;
            iter->prevIsland = island;
            return island;
        }

    }

    return nullptr;
}

void ftIslandSystem::resetIslandID(ftBodyBuffer* buffer) {
    ftBodyBuffer* bodyBuffer = buffer;
    ftBodyBuffer::ftIter iter = bodyBuffer->iterate();
    for (ftBody *body = bodyBuffer->start(&iter); body != nullptr; body = bodyBuffer->next(&iter)) {
        body->islandId = -1;
    }
}

void ftIslandSystem::resetContactFlag() {
    for (uint32 i = 0;i < m_islandContacts.getSize(); ++i) {
        m_islandContacts[i].dfsFlag = false;
    }
}
