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
    m_islandJoints.init(64);
}

void ftIslandSystem::shutdown() {
    m_islandContacts.cleanup();
    m_islandJoints.cleanup();
}

void ftIslandSystem::addContact(ftContact *contact) {

    ftCollider* colliderA = (ftCollider*) contact->userdataA;
    ftCollider* colliderB = (ftCollider*) contact->userdataB;

    ftBody* bodyA = colliderA->body;
    ftBody* bodyB = colliderB->body;

    ftContactEdge* contactEdgeA = createContactEdge(bodyA, bodyB, contact);
    ftContactEdge* contactEdgeB = createContactEdge(bodyB, bodyA, contact);

    int32 index = m_islandContacts.add();
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

void ftIslandSystem::addJoint(ftJoint *joint) {
    ftBody* bodyA = joint->bodyA;
    ftBody* bodyB = joint->bodyB;

    ftJointEdge* jointEdgeA = createJointEdge(bodyA, bodyB, joint);
    ftJointEdge* jointEdgeB = createJointEdge(bodyB, bodyA, joint);


    int32 index = m_islandJoints.add();
    joint->islandIndex = index;
    m_islandJoints[index].joint = joint;
    m_islandJoints[index].jointEdgeA = jointEdgeA;
    m_islandJoints[index].jointEdgeB = jointEdgeB;
}

void ftIslandSystem::removeJoint(ftJoint *joint) {

    ftIslandJoint* islandJoint = &m_islandJoints[joint->islandIndex];
    destroyJointEdge(islandJoint->jointEdgeA);
    destroyJointEdge(islandJoint->jointEdgeB);

    uint32 lastIndex = m_islandJoints.getSize() - 1;
    ftJoint* lastJoint = m_islandJoints[lastIndex].joint;
    lastJoint->islandIndex = joint->islandIndex;
    m_islandJoints[joint->islandIndex] = m_islandJoints[lastIndex];
    m_islandJoints.remove();

}

ftJointEdge* ftIslandSystem::createJointEdge(ftBody* body1, ftBody* body2, ftJoint* joint) {
    ftJointEdge* jointEdge = new ftJointEdge;
    jointEdge->other = body2;

    jointEdge->next = body1->jointList;
    if (body1->jointList != nullptr) body1->jointList->prev = jointEdge;
    jointEdge->prev = nullptr;
    jointEdge->joint = joint;

    body1->jointList = jointEdge;

    return jointEdge;
}

void ftIslandSystem::destroyJointEdge(ftJointEdge *jointEdge) {

    ftJointEdge* prev = jointEdge->prev;
    ftJointEdge* next = jointEdge->next;

    ftJoint* joint = jointEdge->joint;
    ftBody* bodyA = joint->bodyA;
    ftBody* bodyB = joint->bodyB;

    ftBody* body;
    if (bodyA == jointEdge->other) body = bodyB;
    else body = bodyA;

    if (prev!=nullptr) prev->next = next;
    else body->jointList = next;
    if (next!=nullptr) next->prev = prev;

    delete jointEdge;

}

void ftIslandSystem::buildAndProcessIsland(std::function<void(const ftIsland &)> func) {

    resetIslandID(m_buffers.dynamicBuffer);
    resetIslandID(m_buffers.sleepingBuffer);
    resetContactFlag();
    resetJointFlag();

    int nBody  = m_buffers.dynamicBuffer->getSize() + m_buffers.staticBuffer->getSize() +
                 m_buffers.kinematicBuffer->getSize() + m_buffers.sleepingBuffer->getSize();
    ftBodyBuffer *bodyBuffer = m_buffers.dynamicBuffer;
    ftBodyBuffer::ftIter iter = m_buffers.dynamicBuffer->iterate();

    ftBody **dfsStack = new ftBody *[nBody];

    for (ftBody *body = bodyBuffer->start(&iter); body != nullptr; body = bodyBuffer->next(&iter)) {
        if (body->islandId == -1) {

            resetIslandID(m_buffers.staticBuffer);
            resetIslandID(m_buffers.kinematicBuffer);

            int stackSize = 0;

            ftIsland *island = new ftIsland;
            island->bodies.init(64);
            island->contacts.init(64);
            island->joints.init(64);

            dfsStack[0] = body;
            ++stackSize;

            while (stackSize > 0) {
                ftBody *topBody = dfsStack[stackSize - 1];
                --stackSize;

                if (topBody->islandId != -1) continue;

                int index = island->bodies.add();
                island->bodies[index] = topBody;
                topBody->islandId = index;

                if (topBody->bodyType == STATIC) continue;

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

                for (ftJointEdge* jointEdge = topBody->jointList;
                        jointEdge != nullptr; jointEdge = jointEdge->next) {
                    ftJoint* joint = jointEdge->joint;
                    ftIslandJoint* islandJoint = &(m_islandJoints[joint->islandIndex]);
                    if (islandJoint->dfsFlag) {
                        continue;
                    }

                    island->joints.push(joint);
                    islandJoint->dfsFlag = true;

                    dfsStack[stackSize] = jointEdge->other;
                    ++stackSize;
                }
            }

            func(*island);

            island->bodies.cleanup();
            island->contacts.cleanup();
            island->joints.cleanup();
            delete island;
        }

    }

    delete[] dfsStack;
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

void ftIslandSystem::resetJointFlag() {
    for (uint32 i = 0;i < m_islandJoints.getSize(); ++i) {
        m_islandJoints[i].dfsFlag = false;
    }
}