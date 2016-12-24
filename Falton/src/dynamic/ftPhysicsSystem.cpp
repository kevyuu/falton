//
// Created by Kevin Yu on 4/14/16.
//


#include "falton/dynamic/ftPhysicsSystem.h"
#include "falton/dynamic/ftJoint.h"
#include "falton/dynamic/ftIsland.h"

#include "ftProfiler.h"

void ftPhysicsSystem::setConfiguration(const ftConfig& config) {
    m_contactSolver.setConfiguration(config.solverConfig);
    m_collisionSystem.setSleepRatio(config.sleepRatio);
    m_sleepLinearLimit = config.sleepLinearLimit;
    m_sleepAngualrLimit = config.sleepAngularLimit;
    m_sleepTimeLimit = config.sleepTimeLimit;
    m_gravity = config.gravity;
}

void ftPhysicsSystem::init() {

    m_contactSolver.init();

    m_collisionSystem.init(m_broadphase);

    m_dynamicBodies.init();
    m_kinematicBodies.init();
    m_staticBodies.init();
    m_sleepingBodies.init();

    ftBodyBuffers buffers;
    buffers.dynamicBuffer = &m_dynamicBodies;
    buffers.kinematicBuffer = &m_kinematicBodies;
    buffers.staticBuffer = &m_staticBodies;
    buffers.sleepingBuffer = &m_sleepingBodies;
    m_islandSystem.init(buffers);

    m_joints.init(64);

    m_shapeBuffer.init();

}

void ftPhysicsSystem::shutdown() {

    m_contactSolver.shutdown();
    m_collisionSystem.shutdown();
    m_islandSystem.shutdown();

    auto deleteColliders = [] (ftBody* body) {
        for (ftCollider* collider = body->colliders; collider!=nullptr;) {
            ftCollider* next = collider->next;
            delete collider;
            collider = next;
        }
    };

    m_dynamicBodies.forEach(std::cref(deleteColliders));
    m_kinematicBodies.forEach(std::cref(deleteColliders));
    m_staticBodies.forEach(std::cref(deleteColliders));
    m_sleepingBodies.forEach(std::cref(deleteColliders));

    m_dynamicBodies.cleanup();
    m_kinematicBodies.cleanup();
    m_staticBodies.cleanup();
    m_sleepingBodies.cleanup();

    m_joints.cleanup();

    m_shapeBuffer.cleanup();

}

void ftPhysicsSystem::installBroadphase(ftBroadphaseSystem* broadphase) {
    m_broadphase = broadphase;
}

ftBody* ftPhysicsSystem::createStaticBody(const ftVector2& pos, real orientation) {
    ftBody* body= m_staticBodies.create();
    body->mass = real_Infinity;
    body->inverseMass = 0;
    body->moment = real_Infinity;
    body->inverseMoment = 0;
    body->bodyType = STATIC;
    body->transform.center = pos;
    body->transform.rotation.setAngle(orientation);
    return body;
}

ftBody* ftPhysicsSystem::createKinematicBody(const ftVector2& pos, real orientation) {
    ftBody* body= m_kinematicBodies.create();
    body->mass = real_Infinity;
    body->inverseMass = 0;
    body->moment = real_Infinity;
    body->inverseMoment = 0;
    body->bodyType = KINEMATIC;
    body->transform.center = pos;
    body->transform.rotation.setAngle(orientation);
    return body;
}

ftBody* ftPhysicsSystem::createDynamicBody(const ftVector2& pos, real orientation, real mass, real moment) {
    ftAssert(mass != 0, "");
    ftBody* body = m_dynamicBodies.create();
    body->mass = mass;
    body->inverseMass = 1 / mass;
    body->moment = moment;
    body->inverseMoment = 1/moment;
    body->bodyType = DYNAMIC;
    body->transform.center = pos;
    body->transform.rotation.setAngle(orientation);
    return body;
}

void ftPhysicsSystem::updateBody(ftBody* body) {
    ftAssert(body!=nullptr, "");
    if (body->activationState == SLEEP) {
        body->activationState = ACTIVE;
        body->sleepTimer = 0.0f;
        m_sleepingBodies.unlink(body);
        m_dynamicBodies.insert(body);
    }
}

void ftPhysicsSystem::destroyBody(ftBody *body) {
    ftAssert(body!=nullptr, "");
    for (ftContactEdge* cEdge = body->contactList; cEdge != nullptr;) {
        ftContactEdge* next = cEdge->next;
        m_islandSystem.removeContact(cEdge->contact);
        m_collisionSystem.destroyContact(cEdge->contact);
        cEdge = next;
    }

    for (ftCollider* collider = body->colliders; collider != nullptr;) {
        ftCollider* next = collider->next;

        m_collisionSystem.removeShape(collider->collisionHandle);
        destroyShape(collider->shape);
        delete collider;

        collider = next;
    }

    ftBodyBuffer *bodyBuffer;

    if (body->bodyType == STATIC) bodyBuffer = &(m_staticBodies);
    else if (body->bodyType == KINEMATIC) bodyBuffer = &(m_kinematicBodies);
    else bodyBuffer = &(m_dynamicBodies);

    bodyBuffer->destroy(body);

}

ftCollider* ftPhysicsSystem::createCollider(ftBody* body, ftShape* shape,
                                            const ftVector2& position, real orientation) {

    ftAssert(body != nullptr, "");
    ftAssert(shape != nullptr, "");

    ftCollider* collider = new ftCollider;

    collider->transform = ftTransform(position,orientation);

    collider->body = body;

    collider->shape = createShape(shape);

    collider->collisionHandle = m_collisionSystem.addShape(body->transform * collider->transform, collider->shape, collider);

    collider->next = body->colliders;
    body->colliders = collider;

    return collider;

}

void ftPhysicsSystem::destroyCollider(ftCollider* collider) {
    ftAssert(collider!=nullptr, "");
    ftBody* body = collider->body;
    for (ftContactEdge* cEdge = body->contactList; cEdge != nullptr; cEdge = cEdge->next) {
        if (cEdge->contact->userdataA == collider) {
            m_islandSystem.removeContact(cEdge->contact);
            m_collisionSystem.destroyContact(cEdge->contact);
        }
    }

    m_collisionSystem.removeShape(collider->collisionHandle);
    destroyShape(collider->shape);

    ftCollider* prev = nullptr;
    for (ftCollider* current = body->colliders; current != collider;
         prev = current, current = current->next);

    if (prev == nullptr) body->colliders = collider->next;
    else prev->next = collider->next;
}

ftShape* ftPhysicsSystem::createShape(ftShape* shape) {
    switch(shape->shapeType) {
        case SHAPE_CIRCLE: {
            ftCircle* circle = m_shapeBuffer.createCircle();
            circle->copy(shape);
            return circle;
        }
        case SHAPE_POLYGON: {
            ftPolygon* polygon = m_shapeBuffer.createPolygon();
            polygon->copy(shape);
            return polygon;
        }
        default: break;
    }

    return nullptr;
}

void ftPhysicsSystem::destroyShape(ftShape* shape) {
    switch(shape->shapeType) {
        case SHAPE_CIRCLE: {
            ftCircle* circle = (ftCircle*) shape;
            m_shapeBuffer.destoryCircle(circle);
        }
        case SHAPE_POLYGON: {
            ftPolygon* polygon = (ftPolygon*) shape;
            m_shapeBuffer.destroyPolygon(polygon);
        }
        default: break;
    }
}

ftHingeJoint* ftPhysicsSystem::createHingeJoint(ftBody *bodyA, 
                                                ftBody *bodyB, 
                                                ftVector2 anchorPoint) {
    ftAssert(bodyA != nullptr, "");
    ftAssert(bodyB != nullptr, "");
    ftHingeJoint* joint = ftHingeJoint::create(bodyA, bodyB, anchorPoint);
    m_joints.push(joint);
    m_islandSystem.addJoint(joint);
    return joint;
}

ftDistanceJoint* ftPhysicsSystem::createDistanceJoint(ftBody *bodyA, ftBody *bodyB, 
                                                      ftVector2 localAnchorA,
                                                      ftVector2 localAnchorB) {
    ftAssert(bodyA != nullptr, "");
    ftAssert(bodyB != nullptr, "");
    ftDistanceJoint* joint = ftDistanceJoint::create(bodyA, bodyB, localAnchorA, localAnchorB);
    m_joints.push(joint);
    m_islandSystem.addJoint(joint);
    return joint;
}

ftSpringJoint* ftPhysicsSystem::createSpringJoint(ftBody *bodyA, ftBody *bodyB, 
                                                  ftVector2 localAnchorA,
                                                  ftVector2 localAnchorB) {
    ftAssert(bodyA != nullptr, "");
    ftAssert(bodyB != nullptr, "");
    ftSpringJoint* joint = ftSpringJoint::create(bodyA, bodyB, localAnchorA, localAnchorB);
    m_joints.push(joint);
    m_islandSystem.addJoint(joint);
    return joint;
}

ftDynamoJoint* ftPhysicsSystem::createDynamoJoint(ftBody *bodyA, 
                                                  ftBody *bodyB, 
                                                  real targetRate, 
                                                  real maxTorque) {
    ftAssert(bodyA != nullptr, "");
    ftAssert(bodyB != nullptr, "");
    ftDynamoJoint* joint = ftDynamoJoint::create(bodyA, bodyB, targetRate, maxTorque);
    m_joints.push(joint);
    m_islandSystem.addJoint(joint);
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

    const auto updateColSystem = [this](ftBody *body) -> void
    {
        for (auto collider = body->colliders; collider!= nullptr; collider = collider->next) {
            this->m_collisionSystem.moveShape(collider->collisionHandle, body->transform * collider->transform);
        }
    };

    m_dynamicBodies.forEach(std::cref(updateColSystem));
    m_kinematicBodies.forEach(std::cref(updateColSystem));

    const auto colFilter = [] (void* userdataA, void* userdataB) -> bool {

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

    ftCollisionCallback callback;
    callback.beginContact = &beginContactListener;
    callback.updateContact = &updateContactListener;
    callback.endContact = &endContactListener;
    callback.data = &m_islandSystem;

    m_collisionSystem.updateContacts(colFilter, callback);

    const auto processIsland =
            [this,dt]
            (const ftIsland& island) -> void {
        bool allSleeping = true;
        const ftChunkArray<ftBody *> *bodies = &(island.bodies);
        for (uint32 i = 0; i < island.bodies.getSize(); ++i) {
            ftBody* body = (*bodies)[i];
            bool isStatic = body->bodyType == STATIC;
            bool isKinematic = body->bodyType == KINEMATIC;
            bool wantSleep = body->sleepTimer > m_sleepTimeLimit;
            if (isKinematic || (!isStatic && !wantSleep)) {
                allSleeping = false;
                break;
            }
        }

        if (allSleeping) {
            for (uint32 i = 0; i < island.bodies.getSize(); ++i) {
                ftBody* body = (*bodies)[i];
                if (body->bodyType == DYNAMIC && body->activationState != SLEEP) {
                    body->activationState = SLEEP;
                    body->velocity.setZero();
                    body->angularVelocity = 0;
                    this->m_dynamicBodies.unlink(body);
                    this->m_sleepingBodies.insert(body);
                }
            }
        } else {
            for (uint32 i = 0; i < island.bodies.getSize(); ++i) {
                ftBody* body = (*bodies)[i];
                if (body->activationState == SLEEP) {
                    body->activationState = ACTIVE;
                    body->sleepTimer = 0;
                    this->m_sleepingBodies.unlink(body);
                    this->m_dynamicBodies.insert(body);
                }
            }

            this->m_contactSolver.createConstraints(island);
            this->m_contactSolver.preSolve(dt);
            this->m_contactSolver.warmStart();
            this->m_contactSolver.solve(dt);
            this->m_contactSolver.clearConstraints();
        }
    };

    m_islandSystem.buildAndProcessIsland(std::cref(processIsland));

    updateBodiesActivation(dt);

    integratePosition(dt);

}

void ftPhysicsSystem::integrateVelocity(real dt) {

    ftVector2 gravity = m_gravity;
    const auto updateVelo = [gravity, dt] (ftBody* body) {
        body->velocity += gravity * dt;
    };

    m_dynamicBodies.forEach(updateVelo);

}

void ftPhysicsSystem::integratePosition(real dt) {
    const auto updatePos = [dt] (ftBody* body) {
        body->transform.rotation += body->angularVelocity * dt;

        ftVector2 worldCom = body->transform * body->centerOfMass;
        worldCom += body->velocity * dt;
        body->transform.center = worldCom + (body->transform.rotation * (body->centerOfMass * -1));
    };

    m_dynamicBodies.forEach(updatePos);
    m_kinematicBodies.forEach(updatePos);
}

void ftPhysicsSystem::updateBodiesActivation(real dt) {
    auto iter = m_dynamicBodies.iterate();
    for (ftBody* body = m_dynamicBodies.start(&iter);
         body != nullptr; body = m_dynamicBodies.next(&iter)) {
        if ((body->velocity.magnitude() < m_sleepLinearLimit) &&
            ftAbs(body->angularVelocity) < m_sleepAngualrLimit) {
            body->sleepTimer += dt;
        } else {
            body->sleepTimer = 0;
        }
    }
}

void ftPhysicsSystem::beginContactListener(ftContact *contact, void* data) {

    ftIslandSystem* islandSystem = (ftIslandSystem*) data;
    islandSystem->addContact(contact);
}

void ftPhysicsSystem::updateContactListener(ftContact *contact, void *data ) {
    //do nothing
}

void ftPhysicsSystem::endContactListener(ftContact *contact, void *data) {
    ftIslandSystem* islandSystem = (ftIslandSystem*) data;
    islandSystem->removeContact(contact);
}