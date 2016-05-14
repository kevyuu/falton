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

#include <iostream>
using namespace std;

void ftPhysicsSystem::init(ftVector2 gravity) {

    ftContactSolverOption option;
    option.allowedPenetration = 0.01;
    option.baumgarteCoef = 0.2;
    option.numIteration = 10;
    m_contactSolver.init(option);

    m_broadphase = new ftHierarchicalGrid;
    m_collisionSystem.init(m_broadphase);

    ftBodyBuffers buffers;
    buffers.dynamicBuffer = &m_dynamicBodies;
    buffers.kinematicBuffer = &m_kinematicBodies;
    buffers.staticBuffer = &m_staticBodies;
    buffers.sleepingBuffer = &m_sleepingBodies;
    m_islandSystem.init(buffers);

    m_joints.init(64);

    this->m_gravity = gravity;
}

void ftPhysicsSystem::shutdown() {
    m_contactSolver.shutdown();
    m_collisionSystem.shutdown();
    m_islandSystem.shutdown();

    m_joints.cleanup();

    delete m_broadphase;
}

ftBody* ftPhysicsSystem::createBody(const ftBodyDef &bodyDef) {

    ftBodyBuffer *bodyBuffer;

    if (bodyDef.bodyType == STATIC) bodyBuffer = &(m_staticBodies);
    else if (bodyDef.bodyType == KINEMATIC) bodyBuffer = &(m_kinematicBodies);
    else bodyBuffer = &(m_dynamicBodies);

    ftBody* body = bodyBuffer->create();

    body->transform.center = bodyDef.position;
    body->transform.rotation.setAngle(bodyDef.orientation);

    body->velocity = bodyDef.velocity;

    body->centerOfMass = bodyDef.centerOfMass;

    body->angularVelocity = bodyDef.angularVelocity;

    body->bodyType = bodyDef.bodyType;
    body->activationState = ACTIVE;

    body->sleepTimer = 0.0;

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

    if (body->bodyType == STATIC) bodyBuffer = &(m_staticBodies);
    else if (body->bodyType == KINEMATIC) bodyBuffer = &(m_kinematicBodies);
    else bodyBuffer = &(m_dynamicBodies);

    bodyBuffer->destroy(body);

}

ftCollider* ftPhysicsSystem::createCollider(const ftColliderDef &colliderDef) {

    ftCollider* collider = new ftCollider;

    collider->transform = ftTransform(colliderDef.position,colliderDef.orientation);

    collider->body = colliderDef.body;

    collider->friction = colliderDef.friction;
    collider->restitution = colliderDef.restitution;

    collider->group = colliderDef.group;
    collider->category = colliderDef.category;
    collider->mask = colliderDef.mask;

    collider->shape = colliderDef.shape;

    ftBody* body = collider->body;
    collider->collisionHandle = m_collisionSystem.addShape(body->transform * collider->transform, collider->shape, collider);

    collider->next = body->colliders;
    body->colliders = collider;

    return collider;

}

ftPinJoint* ftPhysicsSystem::createJoint(ftBody *bodyA, ftBody *bodyB, ftVector2 anchorPoint) {
    ftPinJoint* joint = ftPinJoint::create(bodyA, bodyB, anchorPoint);

    m_joints.push(joint);

    return joint;
}

void ftPhysicsSystem::iterateBody(ftBodyIterFunc iterFunc, void* data) {
    iterateStaticBody(iterFunc, data);
    iterateKinematicBody(iterFunc, data);
    iterateDynamicBody(iterFunc, data);
}

void ftPhysicsSystem::iterateStaticBody(ftBodyIterFunc iterFunc, void *data) {
    ftBodyBuffer::ftIter iter = m_staticBodies.iterate();
    for (ftBody *body = m_staticBodies.start(&iter); body != nullptr; body = m_staticBodies.next(&iter)) {
        iterFunc(body, data);
    }


}

void ftPhysicsSystem::iterateKinematicBody(ftBodyIterFunc iterFunc, void *data) {
    ftBodyBuffer::ftIter iter = m_kinematicBodies.iterate();
    for (ftBody *body = m_staticBodies.start(&iter); body != nullptr; body = m_kinematicBodies.next(&iter)) {
        iterFunc(body, data);
    }
}

void ftPhysicsSystem::iterateDynamicBody(ftBodyIterFunc iterFunc, void *data) {
    {
        ftBodyBuffer::ftIter iter = m_dynamicBodies.iterate();
        for (ftBody *body = m_dynamicBodies.start(&iter); body != nullptr; body = m_dynamicBodies.next(&iter)) {
            iterFunc(body, data);
        }
    }

    {
        ftBodyBuffer::ftIter iter = m_sleepingBodies.iterate();
        for (ftBody *body = m_sleepingBodies.start(&iter); body != nullptr; body = m_sleepingBodies.next(&iter)) {
            iterFunc(body, data);
        }
    }
}

void ftPhysicsSystem::step(real dt) {

    integrateVelocity(dt);

    ftCollisionCallback callback;
    callback.beginContact = &beginContactListener;
    callback.updateContact = &updateContactListener;
    callback.endContact = &endContactListener;
    callback.data = &m_islandSystem;

    auto colFilter = [] (void* userdataA, void* userdataB) -> bool {
        ftCollider* colliderA = (ftCollider*) userdataA;
        ftCollider* colliderB = (ftCollider*) userdataB;

        ftBody* bodyA = colliderA->body;
        ftBody* bodyB = colliderB->body;

        if (bodyA == bodyB) return false;
        if (colliderA->group != 0 && colliderA->group == colliderB->group) return false;

        int maskA = colliderA->mask;
        int maskB = colliderB->mask;
        int categoryA = colliderA->category;
        int categoryB = colliderB->category;

        if (!(maskA & categoryB)) return false;
        if (!(maskB & categoryA)) return false;

        return true;

    };

    m_collisionSystem.updateContacts(&m_contactBuffer, colFilter, callback);


    ftIslandSystem::ftIter islandIter = m_islandSystem.iterate();
    for (ftIsland* island = m_islandSystem.start(&islandIter); island!=nullptr; island = m_islandSystem.next(&islandIter)) {

        bool allSleeping = true;
        ftChunkArray<ftBody *> *bodies = &(island->bodies);
        for (uint32 i = 0; i < island->bodies.getSize(); ++i) {
            ftBody* body = (*bodies)[i];
            bool isStatic = body->bodyType == STATIC;
            bool isKinematic = body->bodyType == KINEMATIC;
            bool wantSleep = body->sleepTimer > SLEEP_TIME_THRESHOLD;
            if (isKinematic || (!isStatic && !wantSleep)) {
                allSleeping = false;
                break;
            }
        }

        if (allSleeping) {
            for (uint32 i = 0; i < island->bodies.getSize(); ++i) {
                ftBody* body = (*bodies)[i];
                if (body->bodyType == DYNAMIC && body->activationState != SLEEP) {
                    body->activationState = SLEEP;
                    body->velocity.setZero();
                    body->angularVelocity = 0;
                    m_dynamicBodies.unlink(body);
                    m_sleepingBodies.insert(body);
                }
            }
        } else {
            for (uint32 i = 0; i < island->bodies.getSize(); ++i) {
                ftBody* body = (*bodies)[i];
                if (body->activationState == SLEEP) {
                    body->activationState = ACTIVE;
                    body->sleepTimer = 0;
                    m_sleepingBodies.unlink(body);
                    m_dynamicBodies.insert(body);
                }
            }

            m_contactSolver.createConstraints(*island);
            m_contactSolver.warmStart();
            m_contactSolver.solve(dt);
            m_contactSolver.clearConstraints();
        }
    }


    updateBodiesActivation(dt);

    for (uint32 iter = 5 ; iter >0 ; --iter) {
        for (uint32 i = 0; i < m_joints.getSize(); ++i) {
            m_joints[i]->preSolve(dt);
            m_joints[i]->solve();
        }
    }

    integratePosition(dt);

    //update collision system
    {
        ftBodyBuffer::ftIter iter = m_dynamicBodies.iterate();
        for (ftBody* body = m_dynamicBodies.start(&iter); body != nullptr; body = m_dynamicBodies.next(&iter)) {
            for (ftCollider* collider = body->colliders; collider != nullptr; collider = collider->next) {
                m_collisionSystem.moveShape(collider->collisionHandle, body->transform * collider->transform);
            }
        }

        iter = m_kinematicBodies.iterate();
        for (ftBody* body = m_kinematicBodies.start(&iter); body != nullptr; body = m_kinematicBodies.next(&iter)) {

            if (body->activationState == SLEEP) continue;

            for (ftCollider* collider = body->colliders; collider != nullptr; collider = collider->next) {
                m_collisionSystem.moveShape(collider->collisionHandle, body->transform * collider->transform);
            }
        }
    }

}


void ftPhysicsSystem::integrateVelocity(real dt) {

    ftBodyBuffer::ftIter iter = m_dynamicBodies.iterate();
    for (ftBody* body = m_dynamicBodies.start(&iter); body!=nullptr; body = m_dynamicBodies.next(&iter)) {
        body->velocity += m_gravity *dt;
    }

}

void ftPhysicsSystem::integratePosition(real dt) {
    // integrate dyanmic body position
    {
        ftBodyBuffer::ftIter iter = m_dynamicBodies.iterate();
        for (ftBody* body = m_dynamicBodies.start(&iter); body != nullptr; body = m_dynamicBodies.next(&iter)) {
            body->transform.center += body->velocity * dt;
            body->transform.rotation += (body->angularVelocity * dt);
        }
    }

    // integrate kinematic body position
    {
        ftBodyBuffer::ftIter iter = m_kinematicBodies.iterate();
        for (ftBody* body = m_kinematicBodies.start(&iter); body != nullptr; body = m_kinematicBodies.next(&iter)) {
            body->transform.center += body->velocity * dt;
            body->transform.rotation += body->angularVelocity * dt;
        }
    }
}

void ftPhysicsSystem::updateBodiesActivation(real dt) {
    auto iter = m_dynamicBodies.iterate();
    for (ftBody* body = m_dynamicBodies.start(&iter);
         body != nullptr; body = m_dynamicBodies.next(&iter)) {
        updateBodyActivation(body, dt);
    }
}

void ftPhysicsSystem::updateBodyActivation(ftBody* body, real dt) {
    if ((body->velocity.magnitude() < SLEEP_LIN_SPEED_THRESHOLD) &&
        ftAbs(body->angularVelocity) < SLEEP_ANG_SPEED_THRESHOLD) {
        body->sleepTimer += dt;
    } else {
        body->sleepTimer = 0;
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