//
// Created by Kevin Yu on 4/20/16.
//

#include <falton/dynamic/ftIslandSystem.h>
#include <falton/dynamic/ftCollider.h>
#include <falton/dynamic/ftIsland.h>
#include <falton/container/ftStack.h>


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

    int32 index = m_islandContacts.push();
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


    int32 index = m_islandJoints.push();
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

    ftStack<ftBody*> dfsStack;
    dfsStack.init(nBody);

    int32 nIsland = 0;

    for (ftBody *body = bodyBuffer->start(&iter); body != nullptr; body = bodyBuffer->next(&iter)) {
        if (body->islandId == -1) {
            ++nIsland;

            resetIslandID(m_buffers.staticBuffer);
            resetIslandID(m_buffers.kinematicBuffer);

            ftIsland *island = new ftIsland;
            island->bodies.init(64);
            island->contacts.init(64);
            island->joints.init(64);

            dfsStack.push(body);
            body->islandId = -2;

            while (dfsStack.getSize() > 0) {
                ftBody *topBody = dfsStack.pop();

                ftAssert(topBody->islandId == -2, "islandId : "<< topBody->islandId);

                int index = island->bodies.push();
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

                    if (contactEdge->other->islandId != -1) continue;

                    dfsStack.push(contactEdge->other);
                    contactEdge->other->islandId = -2;
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

                    if (jointEdge->other->islandId != -1) continue;

                    dfsStack.push(jointEdge->other);
                    jointEdge->other->islandId = -2;

                }
            }

            func(*island);

            island->bodies.cleanup();
            island->contacts.cleanup();
            island->joints.cleanup();
            delete island;
        }

    }

    dfsStack.cleanup();
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