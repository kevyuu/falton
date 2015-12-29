//
// Created by Kevin Yu on 12/6/15.
//

#include "falton/physics/ftBody.h"

ftBodyTable::ftBodyTable(int initialSize) : bodies(initialSize) {
    handleMap = new ftBody*[initialSize];
    nBody = 0;
    lastId = 0;
}

ftBodyTable::~ftBodyTable() {
    delete handleMap;
}

ftBodyHandle ftBodyTable::create(const ftBodyDef& bodyDef) {
    ftBodyHandle bodyHandle;
    bodyHandle.id = nextID();

    if (nBody > bodies.getCapacity()) {
        bodies.addChunk();
    }

    ftBody *newBody = &(bodies[nBody]);

    handleMap[bodyHandle.id] = newBody;

    newBody->bodyHandle = bodyHandle;

    newBody->position = bodyDef.position;
    newBody->velocity = bodyDef.velocity;
    newBody->orientation = bodyDef.orientation;
    newBody->angular_velocity = bodyDef.angular_velocity;

    newBody->mass = bodyDef.mass;
    newBody->inverse_mass = 1/bodyDef.mass;

    newBody->rotation_inerita = bodyDef.rotation_inertia;
    newBody->inverse_rotation_inertia = 1/bodyDef.rotation_inertia;

    nBody++;

    return bodyHandle;
}

void ftBodyTable::destroy(ftBodyHandle bodyHandle) {

    ftBody* deletedBody = handleMap[bodyHandle.id];
    handleMap[bodyHandle.id] = NULL;
    ftBody* lastBody = &(bodies[nBody]);
    handleMap[lastBody->bodyHandle.id] = deletedBody;
    (*deletedBody) = (*lastBody);

    nBody--;

    freeID.push_back(bodyHandle.id);
}

void ftBodyTable::setPosition(ftBodyHandle bodyHandle, const ftVector2& newPosition) {
    ftBody* body = handleMap[bodyHandle.id];
    body->position = newPosition;
}

void ftBodyTable::setVelocity(ftBodyHandle bodyHandle, const ftVector2& newVelocity) {
    ftBody* body = handleMap[bodyHandle.id];
    body->velocity = newVelocity;
}

void ftBodyTable::setOrientation(ftBodyHandle bodyHandle, const real newOrientation) {
    ftBody* body = handleMap[bodyHandle.id];
    body->orientation = newOrientation;
}

void ftBodyTable::setAngularVelocity(ftBodyHandle bodyHandle, const real newAngularVelocity) {
    ftBody* body = handleMap[bodyHandle.id];
    body->angular_velocity = newAngularVelocity;
}

void ftBodyTable::setMass(ftBodyHandle bodyHandle, const real newMass) {
    ftBody* body = handleMap[bodyHandle.id];
    body->mass = newMass;
    body->inverse_mass = 1/newMass;
}

void ftBodyTable::setRotationInertia(ftBodyHandle bodyHandle, const real newInertia) {
    ftBody* body = handleMap[bodyHandle.id];
    body->rotation_inerita = newInertia;
    body->inverse_rotation_inertia = 1/newInertia;
}

ftVector2 ftBodyTable::getPosition(ftBodyHandle bodyHandle) {
    ftBody* body = handleMap[bodyHandle.id];
    return body->position;
}

ftVector2 ftBodyTable::getVelocity(ftBodyHandle bodyHandle) {
    ftBody* body = handleMap[bodyHandle.id];
    return body->velocity;
}

real ftBodyTable::getOrientation(ftBodyHandle bodyHandle) {
    ftBody* body = handleMap[bodyHandle.id];
    return body->orientation;
}

real ftBodyTable::getAngularVelocity(ftBodyHandle bodyHandle) {
    ftBody* body = handleMap[bodyHandle.id];
    return body->angular_velocity;
}

real ftBodyTable::getMass(ftBodyHandle bodyHandle) {
    ftBody* body = handleMap[bodyHandle.id];
    return body->mass;
}

real ftBodyTable::getInverseMass(ftBodyHandle bodyHandle) {
    ftBody* body = handleMap[bodyHandle.id];
    return body->inverse_mass;
}

real ftBodyTable::getRotationInertia(ftBodyHandle bodyHandle) {
    ftBody* body = handleMap[bodyHandle.id];
    return body->rotation_inerita;
}

real ftBodyTable::getInverseRotationInertia(ftBodyHandle bodyHandle) {
    ftBody* body = handleMap[bodyHandle.id];
    return body->inverse_rotation_inertia;
}

int ftBodyTable::nextID() {
    if (freeID.size() > 0) {
        int next = freeID.front();
        freeID.pop_front();
        return next;
    }
    return lastId++;
}
