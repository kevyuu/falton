//
// Created by Kevin Yu on 3/22/16.
//

#include "falton/physics/ftContactSolver.h"
#include <iostream>

using namespace std;

void ftContactSolver::createContactConstraint(ftCollider* colliderA, ftCollider* colliderB, ftManifold *manifold, ftContactConstraint* constraint) {
    ftBody *bodyA = colliderA->body;
    ftBody *bodyB = colliderB->body;

    constraint->bodyA = bodyA;
    constraint->bodyB = bodyB;
    constraint->normal = manifold->normal;
    constraint->numContactPoint = manifold->numContact;
    constraint->frictionCoef = real_sqrt(colliderA->friction * colliderB->friction);

    ftVector2 oneVector(1,1);

    for (int i=0;i<constraint->numContactPoint;i++) {
        ftContactPointConstraint *pointConstraint = &(constraint->pointConstraint[i]);

        pointConstraint->r1 = manifold->contactPoints[i].r1 - bodyA->transform.center;
        pointConstraint->r2 = manifold->contactPoints[i].r2 - bodyB->transform.center;

        float kNormal = bodyA->inverseMass + bodyB->inverseMass;

        float rnA = pointConstraint->r1.cross(constraint->normal);
        float rnB = pointConstraint->r2.cross(constraint->normal);

        kNormal += (bodyA->inverseMoment * rnA * rnA + bodyB->inverseMoment * rnB * rnB);

        pointConstraint->normalMass = 1/kNormal;

        float kTangent = bodyA->inverseMass + bodyB->inverseMass;
        ftVector2 tangent = constraint->normal.perpendicular();
        float rtA = pointConstraint->r1.cross(tangent);
        float rtB = pointConstraint->r2.cross(tangent);

        kTangent += (bodyA->inverseMoment * rtA * rtA + bodyB->inverseMoment * rtB * rtB);
        pointConstraint->tangentMass = 1/kTangent;

        pointConstraint->accumNormalImpulse = 0;
        pointConstraint->accumTangentImpulse = 0;

        real slop = manifold->penetrationDepth[i] - option.allowedPenetration;
        if (slop < 0) slop = 0;
        pointConstraint->positionBias = option.baumgarteCoef * slop;

    }
}

ftContactSolver::ftContactSolver(const ftContactSolverOption& option) : option(option) {
    constraintBuffer = new ftChunkArray<ftContactConstraint>(64);
    constraintCache = new ftChunkArray<ftContactConstraint>(64);
}

ftContactSolver::~ftContactSolver() {
    delete constraintBuffer;
    delete constraintCache;
}

void ftContactSolver::addConstraint(ftCollider* colliderA, ftCollider* colliderB, ftContact* contact) {

    int pos = constraintBuffer->add();
    createContactConstraint(colliderA, colliderB, &(contact->manifold) , &((*constraintBuffer)[pos]));
    contact->constraint = (void *)&((*constraintBuffer)[pos]);

}

void ftContactSolver::updateConstraint(ftCollider* colliderA, ftCollider* colliderB, ftContact* contact) {

    ftContactConstraint *oldConstraint = (ftContactConstraint*) contact->constraint;

    int pos = constraintBuffer->add();
    createContactConstraint(colliderA, colliderB, &(contact->manifold), &((*constraintBuffer)[pos]));
    ftContactConstraint &newConstraint = (*constraintBuffer)[pos];

    if (oldConstraint -> numContactPoint == newConstraint.numContactPoint) {
        for (int i=0;i<newConstraint.numContactPoint;i++) {
            //newConstraint.pointConstraint[i].accumNormalImpulse = oldConstraint->pointConstraint[i].accumNormalImpulse;
            //newConstraint.pointConstraint[i].accumTangentImpulse = oldConstraint->pointConstraint[i].accumTangentImpulse;
        }

    }

    contact->constraint = &newConstraint;

}

void ftContactSolver::warmStart() {
    for (uint32 i = 0;i < constraintBuffer->getSize(); ++i) {
        ftContactConstraint *constraint = &((*constraintBuffer)[i]);
        ftBody *bodyA = ((*constraintBuffer)[i]).bodyA;
        ftBody *bodyB = ((*constraintBuffer)[i]).bodyB;

        ftVector2 normal = constraint->normal;
        ftVector2 tangent = normal.perpendicular();
        for (uint8 j = 0; j < constraint->numContactPoint; ++j) {
            ftContactPointConstraint *pc = &(constraint->pointConstraint[i]);
            ftVector2 linearI = normal * pc->accumNormalImpulse + tangent * pc->accumTangentImpulse;

            bodyA->velocity -= linearI * bodyA->inverseMass;
            bodyB->velocity += linearI * bodyA->inverseMass;

            bodyA->angularVelocity -= pc->r1.cross(linearI) * bodyA->inverseMoment;
            bodyB->angularVelocity += pc->r2.cross(linearI) * bodyB->inverseMoment;
        }
    }
}

void ftContactSolver::solve(real dt) {

    uint8 numIteration = option.numIteration;
    while (numIteration) {

        for (uint32 i = 0; i < constraintBuffer->getSize(); ++i) {

            ftContactConstraint *constraint = &((*constraintBuffer)[i]);
            ftBody *bodyA = ((*constraintBuffer)[i]).bodyA;
            ftBody *bodyB = ((*constraintBuffer)[i]).bodyB;


            ftVector2 normal = constraint->normal;
            ftVector2 tangent = normal.tangent();

            for (uint8 j = 0; j<constraint->numContactPoint; ++j) {

                ftContactPointConstraint *pointConstraint = &(constraint->pointConstraint[j]);

                ftVector2 vA = bodyA->velocity;
                ftVector2 vB = bodyB->velocity;
                float wA = bodyA->angularVelocity;
                float wB = bodyB->angularVelocity;

                //normal impulse
                {

                    float rnA = pointConstraint->r1.cross(normal);
                    float rnB = pointConstraint->r2.cross(normal);

                    float jnV = vB.dot(normal) + wB * rnB - vA.dot(normal) - wA * rnA;
                    float nLambda = (-jnV + pointConstraint->positionBias/dt) * pointConstraint->normalMass;

                    float oldAccumI = pointConstraint->accumNormalImpulse;
                    pointConstraint->accumNormalImpulse += nLambda;
                    if (pointConstraint->accumNormalImpulse < 0) {
                        pointConstraint->accumNormalImpulse = 0;
                    }
                    float I = pointConstraint->accumNormalImpulse - oldAccumI;

                    ftVector2 nLinearI = normal * I;
                    float nAngularIA = rnA * I;
                    float nAngularIB = rnB * I;

                    bodyA->velocity -= bodyA->inverseMass * nLinearI;
                    bodyB->velocity += bodyB->inverseMass * nLinearI;

                    bodyA->angularVelocity -= bodyA->inverseMoment * nAngularIA;
                    bodyB->angularVelocity += bodyB->inverseMoment * nAngularIB;

                }

                vA = bodyA->velocity;
                vB = bodyB->velocity;
                wA = bodyA->angularVelocity;
                wB = bodyB->angularVelocity;

                //tangent impulse
                {
                    float rtA = pointConstraint->r1.cross(tangent);
                    float rtB = pointConstraint->r2.cross(tangent);
                    float jtV = vB.dot(tangent) + wB * rtB - vA.dot(tangent) - wA * rtA;

                    float nLambda = -jtV * pointConstraint->tangentMass;

                    float oldAccumI = pointConstraint->accumTangentImpulse;
                    pointConstraint->accumTangentImpulse += nLambda;

                    float frictionBound = constraint->frictionCoef * pointConstraint->accumNormalImpulse;
                    if (pointConstraint->accumTangentImpulse > frictionBound) {
                        pointConstraint->accumTangentImpulse = frictionBound;
                    } else if (pointConstraint->accumTangentImpulse < -frictionBound) {
                        pointConstraint->accumTangentImpulse = -frictionBound;
                    }
                    float I = pointConstraint->accumTangentImpulse - oldAccumI;

                    ftVector2 nLinearI = tangent * I;
                    float nAngularIA = rtA * I;
                    float nAngularIB = rtB * I;

                    bodyA->velocity -= bodyA->inverseMass * nLinearI;
                    bodyB->velocity += bodyB->inverseMass * nLinearI;

                    bodyA->angularVelocity -= bodyA->inverseMoment * nAngularIA;
                    bodyB->angularVelocity += bodyB->inverseMoment * nAngularIB;


                }

            }

        }
        numIteration --;

    }

    delete constraintCache;
    constraintCache = constraintBuffer;
    constraintBuffer = new ftChunkArray<ftContactConstraint>(64);

}

