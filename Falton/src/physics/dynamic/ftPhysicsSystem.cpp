//
// Created by Kevin Yu on 4/14/16.
//

#include <falton/physics/joint/ftPinJoint.h>
#include "falton/physics/dynamic/ftPhysicsSystem.h"
#include "falton/physics/dynamic/ftBody.h"
#include <falton/physics/Collision/BroadPhase/ftHierarchicalGrid.h>
#include "falton/math/math.h"
#include "falton/physics/dynamic/ftIsland.h"
#include "falton/physics/dynamic/ftIslandSystem.h"

void ftPhysicsSystem::init(ftVector2 gravity) {

    ftContactSolverOption option;
    option.allowedPenetration = 0.01;
    option.baumgarteCoef = 0.2;
    option.numIteration = 10;
    contactSolver = new ftContactSolver;
    contactSolver->init(option);

    broadphase = new ftHierarchicalGrid;
    collisionSystem = new ftCollisionSystem;
    collisionSystem->init(broadphase);

    ftBodyBuffers buffers;
    buffers.dynamicBuffer = &dynamicBodies;
    buffers.kinematicBuffer = &kinematicBodies;
    buffers.staticBuffer = &staticBodies;
    islandSystem = new ftIslandSystem;
    islandSystem->init(buffers);

    joints = new ftChunkArray<ftPinJoint*>(64);

    this->gravity = gravity;
}

void ftPhysicsSystem::shutdown() {
    delete contactSolver;

    collisionSystem->shutdown();
    delete collisionSystem;

    islandSystem->shutdown();
    delete islandSystem;

    delete joints;

    delete broadphase;
    if (contactConstraint) {
        delete[] contactConstraint;
    }
}

ftBody* ftPhysicsSystem::createBody(const ftBodyDef &bodyDef) {

    ftBodyBuffer *bodyBuffer;

    if (bodyDef.bodyType == STATIC) bodyBuffer = &(staticBodies);
    else if (bodyDef.bodyType == KINEMATIC) bodyBuffer = &(kinematicBodies);
    else bodyBuffer = &(dynamicBodies);

    ftBody* body = bodyBuffer->create();

    body->transform.center = bodyDef.position;
    body->transform.rotation.setAngle(bodyDef.orientation);

    body->velocity = bodyDef.velocity;

    body->centerOfMass = bodyDef.centerOfMass;

    body->angularVelocity = bodyDef.angularVelocity;

    body->bodyType = bodyDef.bodyType;

    if (body->bodyType == DYNAMIC) {
        body->mass = bodyDef.mass;
        body->inverseMass = 1 / bodyDef.mass;
        body->moment = bodyDef.moment;
        body->inverseMoment = 1 / bodyDef.moment;
    } else {
        body->mass = real_Infinity;
        body->inverseMass = 0;
        body->moment = real_Infinity;
        body->inverseMoment = 0;
    }

    body->colliders = nullptr;

    return body;
}

void ftPhysicsSystem::destroyBody(ftBody *body) {

    ftBodyBuffer *bodyBuffer;

    if (body->bodyType == STATIC) bodyBuffer = &(staticBodies);
    else if (body->bodyType == KINEMATIC) bodyBuffer = &(kinematicBodies);
    else bodyBuffer = &(dynamicBodies);

    bodyBuffer->destroy(body);

}

ftCollider* ftPhysicsSystem::createCollider(const ftColliderDef &colliderDef) {

    ftCollider* collider = new ftCollider;

    collider->transform = ftTransform(colliderDef.position,colliderDef.orientation);

    collider->body = colliderDef.body;

    collider->friction = colliderDef.friction;
    collider->restitution = colliderDef.restitution;

    collider->shape = colliderDef.shape;

    ftBody* body = collider->body;
    collider->collisionHandle = collisionSystem->addShape(body->transform * collider->transform, collider->shape, collider);

    collider->next = body->colliders;
    body->colliders = collider;

    return collider;

}

ftPinJoint* ftPhysicsSystem::createJoint(ftBody *bodyA, ftBody *bodyB, ftVector2 anchorPoint) {
    ftPinJoint* joint = ftPinJoint::create(bodyA, bodyB, anchorPoint);

    joints->push(joint);

    return joint;
}

void ftPhysicsSystem::step(real dt) {

    integrateVelocity(dt);

    ftCollisionCallback callback;
    callback.beginContact = &beginContactListener;
    callback.updateContact = &updateContactListener;
    callback.endContact = &endContactListener;
    callback.data = islandSystem;
    collisionSystem->updateContacts(&contactBuffer, &collisionFilter, callback);

    ftIslandSystem::ftIter islandIter = islandSystem->iterate();
    for (ftIsland* island = islandSystem->start(&islandIter); island!=nullptr; island = islandSystem->next(&islandIter)) {
        contactSolver->createConstraints(island);
        contactSolver->warmStart();
        contactSolver->solve(dt);
        contactSolver->clearConstraints();
    }

    for (uint32 iter = 5 ; iter >0 ; --iter) {
        for (uint32 i = 0; i < joints->getSize(); ++i) {
            (*joints)[i]->preSolve(dt);
            (*joints)[i]->solve();
        }
    }

    integratePosition(dt);

    //update collision system
    {
        ftBodyBuffer::ftIter iter = dynamicBodies.iterate();
        for (ftBody* body = dynamicBodies.start(&iter); body != nullptr; body = dynamicBodies.next(&iter)) {
            for (ftCollider* collider = body->colliders; collider != nullptr; collider = collider->next) {
                collisionSystem->moveShape(collider->collisionHandle, body->transform * collider->transform);
            }
        }

        iter = kinematicBodies.iterate();
        for (ftBody* body = kinematicBodies.start(&iter); body != nullptr; body = kinematicBodies.next(&iter)) {
            for (ftCollider* collider = body->colliders; collider != nullptr; collider = collider->next) {
                collisionSystem->moveShape(collider->collisionHandle, body->transform * collider->transform);
            }
        }
    }

}


void ftPhysicsSystem::integrateVelocity(real dt) {

    ftBodyBuffer::ftIter iter = dynamicBodies.iterate();
    for (ftBody* body = dynamicBodies.start(&iter); body!=nullptr; body = dynamicBodies.next(&iter)) {
        body->velocity += gravity *dt;
    }

}

void ftPhysicsSystem::integratePosition(real dt) {
    // integrate dyanmic body position
    {
        ftBodyBuffer::ftIter iter = dynamicBodies.iterate();
        for (ftBody* body = dynamicBodies.start(&iter); body != nullptr; body = dynamicBodies.next(&iter)) {
            body->transform.center += body->velocity * dt;
            body->transform.rotation += (body->angularVelocity * dt);
        }
    }

    // integrate kinematic body position
    {
        ftBodyBuffer::ftIter iter = kinematicBodies.iterate();
        for (ftBody* body = kinematicBodies.start(&iter); body != nullptr; body = kinematicBodies.next(&iter)) {
            body->transform.center += body->velocity * dt;
            body->transform.rotation += body->angularVelocity * dt;
        }
    }
}

void ftPhysicsSystem::beginContactListener(ftContact *contact, void* data) {
    ftIslandSystem* islandSystem = (ftIslandSystem*) data;
    islandSystem->addContact(contact);
}

void ftPhysicsSystem::updateContactListener(ftContact *contact __attribute__((unused)), void *data __attribute__((unused))) {
    //do nothing
}

void ftPhysicsSystem::endContactListener(ftContact *contact, void *data) {
    ftIslandSystem* islandSystem = (ftIslandSystem*) data;
    islandSystem->removeContact(contact);
}

bool ftPhysicsSystem::collisionFilter(void *userdataA, void *userdataB) {

    ftCollider* colliderA = (ftCollider*) userdataA;
    ftCollider* colliderB = (ftCollider*) userdataB;

    ftBody* bodyA = colliderA->body;
    ftBody* bodyB = colliderB->body;

    if (bodyA == bodyB) return false;

    return true;

}


