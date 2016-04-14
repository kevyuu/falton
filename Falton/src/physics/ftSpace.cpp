//
// Created by Kevin Yu on 12/26/15.
//

#include "falton/physics/ftSpace.h"
#include "falton/physics/joint/ftPinJoint.h"
#include <iostream>

using namespace std;

ftSpace::ftSpace(ftVector2 gravity) : gravity(gravity), staticBodies(nullptr),
                                      kinematicBodies(nullptr), dynamicBodies(nullptr) {

    collisionSystem.init(&nSquaredBroadphase);
    ftContactSolverOption option;
    option.allowedPenetration = 0.01;
    option.baumgarteCoef = 0.2;
    option.numIteration = 10;
    contactSolver = new ftContactSolver(option);
}

ftSpace::~ftSpace() {
    delete contactSolver;
}

ftBody* ftSpace::createBody(const ftBodyDef &bodyDef) {
    ftBody* body = new ftBody(bodyDef);

    ftBody **bodyList;
    if (bodyDef.bodyType == STATIC) {
        bodyList = &staticBodies;
    } else if (bodyDef.bodyType == KINEMATIC) {
        bodyList = &kinematicBodies;
    } else if (bodyDef.bodyType == DYNAMIC) {
        bodyList = &dynamicBodies;
    }

    body->next = *bodyList;
    if ((*bodyList) != nullptr) (*bodyList)->prev = body;
    *bodyList = body;

    return body;
}

void ftSpace::destroyBody(ftBody *body) {

    ftBody* prevBody = body->prev;
    ftBody* nextBody = body->next;

    if (prevBody!=nullptr) prevBody->next = nextBody;
    if (nextBody!=nullptr) nextBody->prev = prevBody;

    delete body;

}

void ftSpace::iterateBody(ftBodyIterFunc iterFunc, void *data) {

    ftBody *staticBody = staticBodies;
    while (staticBody) {
        iterFunc(staticBody,data);
        staticBody = staticBody->next;
    }

    ftBody *dynamicBody = dynamicBodies;
    while (dynamicBody) {
        iterFunc(dynamicBody,data);
        dynamicBody = dynamicBody->next;
    }

    ftBody *kinematicBody = kinematicBodies;
    while (kinematicBody) {
        iterFunc(kinematicBody,data);
        kinematicBody = kinematicBody->next;
    }

}

ftCollider* ftSpace::createCollider(const ftColliderDef &colliderDef) {
    ftCollider *newCollider = new ftCollider(colliderDef);

    ftBody *body = colliderDef.body;
    body->addCollider(newCollider);

    newCollider->collisionHandle = collisionSystem.addShape(body->transform * newCollider->transform,
                                                            newCollider->shape, newCollider);

    return newCollider;

}

ftPinJoint* ftSpace::createJoint(ftBody *bodyA, ftBody *bodyB, ftVector2 anchorPoint) {

    ftPinJoint* joint = ftPinJoint::create(bodyA, bodyB, anchorPoint);



    return joint;

}

void ftSpace::step(real dt) {

    CollisionCallback callback;
    callback.beginContact = &beginContactListener;
    callback.updateContact = &updateContactListener;
    callback.endContact = &endContactListener;
    callback.data = contactSolver;

    // integrate velocity
    {
        ftBody *dynamicBody = dynamicBodies;
        while (dynamicBody) {
            dynamicBody->velocity += gravity * dt;
            dynamicBody = dynamicBody->next;
        }
    }

    collisionSystem.updateContacts(callback);
    //contactSolver->warmStart();
    contactSolver->solve(dt);

    // integrate position
    {
        ftBody *dynamicBody = dynamicBodies;
        while (dynamicBody) {
            dynamicBody->transform.center += dynamicBody->velocity * dt;
            dynamicBody->transform.rotation += (dynamicBody->angularVelocity * dt);
            dynamicBody = dynamicBody->next;
        }

        ftBody *kinematicBody = kinematicBodies;
        while (kinematicBody) {
            kinematicBody->transform.center += kinematicBody->velocity * dt;
            kinematicBody->transform.rotation += (kinematicBody->angularVelocity * dt);
            kinematicBody = kinematicBody->next;
        }
    }

    //update collision system
    {
        ftBody *dynamicBody = dynamicBodies;
        while (dynamicBody) {

            ftCollider *collider = dynamicBody->colliders;
            while(collider) {
                collisionSystem.moveShape(collider->collisionHandle, dynamicBody->transform * collider->transform);
                collider = collider->next;
            }

            dynamicBody = dynamicBody->next;

        }

        ftBody *kinematicBody = kinematicBodies;
        while (kinematicBody) {

            ftCollider *collider = kinematicBody->colliders;
            while(collider) {
                collisionSystem.moveShape(collider->collisionHandle, kinematicBody->transform * collider->transform);
                collider = collider->next;
            }

            kinematicBody = kinematicBody->next;
        }
    }

}



void beginContactListener(void *userDataA, void *userDataB, ftContact* contact, void *data) {

    ftContactSolver *contactSolver = (ftContactSolver*) data;

    ftCollider* colliderA = (ftCollider*) userDataA;
    ftCollider* colliderB = (ftCollider*) userDataB;

    contactSolver->addConstraint(colliderA, colliderB, contact);
}

void updateContactListener(void *userDataA, void *userDataB, ftContact *contact, void *data) {

    ftContactSolver *contactSolver = (ftContactSolver*) data;

    ftCollider* colliderA = (ftCollider*) userDataA;
    ftCollider* colliderB = (ftCollider*) userDataB;

    contactSolver->updateConstraint(colliderA, colliderB, contact);
}

void endContactListener(void *userDataA, void *userDataB, ftContact *contact, void *data) {
}