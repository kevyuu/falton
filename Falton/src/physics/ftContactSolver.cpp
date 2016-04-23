//
// Created by Kevin Yu on 3/22/16.
//

#include "falton/physics/ftContactSolver.h"
#include "falton/physics/dynamic/ftContactConstraint.h"
#include <iostream>
#include <falton/physics/dynamic/ftIsland.h>

using namespace std;

void ftContactSolver::createContactConstraint(ftCollider* colliderA, ftCollider* colliderB,
                                              ftContact* contact, ftContactConstraint* constraint) {

    ftManifold *manifold = &(contact->manifold);

    ftBody *bodyA = colliderA->body;
    ftBody *bodyB = colliderB->body;

    constraint->bodyIDA = bodyA->islandId;
    constraint->bodyIDB = bodyB->islandId;
    constraint->normal = manifold->normal;
    constraint->numContactPoint = manifold->numContact;
    constraint->contact = contact;
    constraint->invMassA = bodyA->inverseMass;
    constraint->invMassB = bodyB->inverseMass;
    constraint->invMomentA = bodyA->inverseMoment;
    constraint->invMomentB = bodyB->inverseMoment;

    constraint->frictionCoef = real_sqrt(colliderA->friction * colliderB->friction);
    constraint->restitution = (colliderA->restitution + colliderB->restitution) / 2;

    for (int i=0;i<constraint->numContactPoint;i++) {
        ftContactPointConstraint *pointConstraint = &(constraint->pointConstraint[i]);

        pointConstraint->r1 = manifold->contactPoints[i].r1 - bodyA->transform.center;
        pointConstraint->r2 = manifold->contactPoints[i].r2 - bodyB->transform.center;

        real kNormal = bodyA->inverseMass + bodyB->inverseMass;

        real rnA = pointConstraint->r1.cross(constraint->normal);
        real rnB = pointConstraint->r2.cross(constraint->normal);

        kNormal += (bodyA->inverseMoment * rnA * rnA + bodyB->inverseMoment * rnB * rnB);

        pointConstraint->normalMass = 1/kNormal;

        real kTangent = bodyA->inverseMass + bodyB->inverseMass;
        ftVector2 tangent = constraint->normal.perpendicular();
        real rtA = pointConstraint->r1.cross(tangent);
        real rtB = pointConstraint->r2.cross(tangent);

        kTangent += (bodyA->inverseMoment * rtA * rtA + bodyB->inverseMoment * rtB * rtB);
        pointConstraint->tangentMass = 1/kTangent;

        real slop = manifold->penetrationDepth[i] - option.allowedPenetration;
        if (slop < 0) slop = 0;
        pointConstraint->positionBias = option.baumgarteCoef * slop;

    }
}

void ftContactSolver::init(const ftContactSolverOption &option){
    this->option = option;
}

void ftContactSolver::shutdown() {
    //do nothing
}

void ftContactSolver::warmStart() {
    for (uint32 i = 0;i<constraintGroup.numConstraint;++i) {
        ftContactConstraint *constraint = &(constraintGroup.constraints[i]);

        uint32 bodyIDA = constraint->bodyIDA;
        uint32 bodyIDB = constraint->bodyIDB;
        ftVector2 normal = constraint->normal;
        ftVector2 tangent = normal.tangent();

        ftManifold* manifold = &(constraint->contact->manifold);

        for (uint32 j = 0;j < constraint->numContactPoint; ++j) {

            ftContactPointConstraint* pointConstraint = &(constraint->pointConstraint[j]);

            pointConstraint->nIAcc = manifold->contactPoints[j].nIAcc;
            pointConstraint->tIAcc = manifold->contactPoints[j].tIAcc;

            ftVector2 impulse = pointConstraint->nIAcc * normal;
            impulse += pointConstraint->tIAcc * tangent;

            constraintGroup.velocities[bodyIDA] -= constraint->invMassA * impulse;
            constraintGroup.velocities[bodyIDB] += constraint->invMassB * impulse;

            ftVector2 r1 = pointConstraint->r1;
            ftVector2 r2 = pointConstraint->r2;
            constraintGroup.angularVelocities[bodyIDA] -= constraint->invMomentA * r1.cross(impulse);
            constraintGroup.angularVelocities[bodyIDB] += constraint->invMomentB * r2.cross(impulse);
        }
    }
}

void ftContactSolver::solve(real dt) {
    uint8 numIteration = option.numIteration;
    while (numIteration > 0) {

        for (uint32 i = 0; i < constraintGroup.numConstraint; ++i) {

            ftContactConstraint *constraint = &(constraintGroup.constraints[i]);
            uint32 bodyIDA = constraint->bodyIDA;
            uint32 bodyIDB = constraint->bodyIDB;
            ftVector2 normal = constraint->normal;
            ftVector2 tangent = normal.tangent();

            for (uint8 j = 0; j<constraint->numContactPoint; ++j) {

                ftContactPointConstraint *pointConstraint = &(constraint->pointConstraint[j]);

                //normal impulse
                {
                    ftVector2 vA = constraintGroup.velocities[bodyIDA];
                    ftVector2 vB = constraintGroup.velocities[bodyIDB];
                    real wA = constraintGroup.angularVelocities[bodyIDA];
                    real wB = constraintGroup.angularVelocities[bodyIDB];

                    ftVector2 dv = (vB + pointConstraint->r2.invCross(wB) - vA - pointConstraint->r1.invCross(wA));
                    real jnV = dv.dot(normal);
                    real restitutionBias = constraint->restitution * jnV;
                    real nLambda = (-jnV + pointConstraint->positionBias/dt + restitutionBias) * pointConstraint->normalMass;

                    real oldAccumI = pointConstraint->nIAcc;
                    pointConstraint->nIAcc += nLambda;
                    if (pointConstraint->nIAcc < 0) {
                        pointConstraint->nIAcc = 0;
                    }
                    real I = pointConstraint->nIAcc - oldAccumI;

                    ftVector2 nLinearI = normal * I;

                    real rnA = pointConstraint->r1.cross(normal);
                    real rnB = pointConstraint->r2.cross(normal);
                    real nAngularIA = rnA * I;
                    real nAngularIB = rnB * I;

                    constraintGroup.velocities[bodyIDA] -= constraint->invMassA * nLinearI;
                    constraintGroup.velocities[bodyIDB] += constraint->invMassB * nLinearI;

                    constraintGroup.angularVelocities[bodyIDA] -= constraint->invMomentA * nAngularIA;
                    constraintGroup.angularVelocities[bodyIDB] += constraint->invMomentB * nAngularIB;

                }

                //tangent impulse
                {
                    ftVector2 vA = constraintGroup.velocities[bodyIDA];
                    ftVector2 vB = constraintGroup.velocities[bodyIDB];
                    real wA = constraintGroup.angularVelocities[bodyIDA];
                    real wB = constraintGroup.angularVelocities[bodyIDB];

                    ftVector2 dv = (vB + pointConstraint->r2.invCross(wB) - vA - pointConstraint->r1.invCross(wA));
                    real jtV = dv.dot(tangent);
                    real tLambda = -jtV * pointConstraint->tangentMass;

                    real oldAccumI = pointConstraint->tIAcc;
                    pointConstraint->tIAcc += tLambda;

                    real frictionBound = constraint->frictionCoef * pointConstraint->nIAcc;
                    if (pointConstraint->tIAcc > frictionBound) {
                        pointConstraint->tIAcc = frictionBound;
                    } else if (pointConstraint->tIAcc < -frictionBound) {
                        pointConstraint->tIAcc = -frictionBound;
                    }
                    real I = pointConstraint->tIAcc - oldAccumI;

                    ftVector2 nLinearI = tangent * I;
                    real rtA = pointConstraint->r1.cross(tangent);
                    real rtB = pointConstraint->r2.cross(tangent);
                    real nAngularIA = rtA * I;
                    real nAngularIB = rtB * I;

                    constraintGroup.velocities[bodyIDA] -= constraint->invMassA * nLinearI;
                    constraintGroup.velocities[bodyIDB] += constraint->invMassB * nLinearI;

                    constraintGroup.angularVelocities[bodyIDA] -= constraint->invMomentA * nAngularIA;
                    constraintGroup.angularVelocities[bodyIDB] += constraint->invMomentB * nAngularIB;

                }

            }

        }
        numIteration --;
    }

    for (uint32 i = 0;i < constraintGroup.numBody; i++) {
        constraintGroup.bodies[i]->velocity = constraintGroup.velocities[i];
        constraintGroup.bodies[i]->angularVelocity = constraintGroup.angularVelocities[i];
    }

    for (uint32 i = 0; i < constraintGroup.numConstraint; ++i) {
        ftContact* contact = constraintGroup.constraints[i].contact;
        ftManifold* manifold = &(contact->manifold);
        ftContactConstraint* constraint = &(constraintGroup.constraints[i]);
        for (uint32 j = 0; j < contact->manifold.numContact; ++j) {
            manifold->contactPoints[j].nIAcc =
                    constraint->pointConstraint[j].nIAcc;
            manifold->contactPoints[j].tIAcc =
                    constraint->pointConstraint[j].tIAcc;
        }
    }
}

void ftContactSolver::createConstraints(const ftIsland *island) {

    int nBody = island->bodies.getSize();
    int nContact = island->contacts.getSize();

    constraintGroup.positions = new ftVector2[nBody];
    constraintGroup.velocities = new ftVector2[nBody];
    constraintGroup.angularVelocities = new real[nBody];
    constraintGroup.bodies = new ftBody*[nBody];
    constraintGroup.constraints = new ftContactConstraint[nContact];

    for (int i=0;i<nBody;i++) {
        ftBody* body = island->bodies[i];
        ftVector2 haha = body->transform.center;
        constraintGroup.positions[i] = haha;
        constraintGroup.velocities[i] = island->bodies[i]->velocity;
        constraintGroup.angularVelocities[i] = island->bodies[i]->angularVelocity;
        constraintGroup.bodies[i] = island->bodies[i];
    }

    for (int i=0;i<nContact;i++) {
        ftContact* contact = island->contacts[i];

        ftCollider* colliderA = (ftCollider*) contact->userdataA;
        ftCollider* colliderB = (ftCollider*) contact->userdataB;
        createContactConstraint(colliderA, colliderB, contact, &(constraintGroup.constraints[i]));

    }

    constraintGroup.numConstraint = nContact;
    constraintGroup.numBody = nBody;

}

void ftContactSolver::clearConstraints() {
    delete[] constraintGroup.positions;
    delete[] constraintGroup.velocities;
    delete[] constraintGroup.angularVelocities;
    delete[] constraintGroup.constraints;
}
