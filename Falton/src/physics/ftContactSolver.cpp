//
// Created by Kevin Yu on 3/22/16.
//

#include <iostream>
#include <falton/physics/dynamic/ftIsland.h>
#include <falton/physics/collision/ftContact.h>
#include <falton/physics/ftContactSolver.h>
#include <falton/physics/dynamic/ftContactConstraint.h>
#include <falton/physics/dynamic/ftBody.h>
#include <falton/physics/dynamic/ftCollider.h>

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
    real restitution = colliderA->restitution > colliderB->restitution ? colliderA->restitution : colliderB->restitution;

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

        real slop = manifold->penetrationDepth[i] - m_option.allowedPenetration;
        if (slop < 0) slop = 0;
        pointConstraint->positionBias = m_option.baumgarteCoef * slop;

        ftVector2 vA = bodyA->velocity;
        ftVector2 vB = bodyB->velocity;
        real wA = bodyA->angularVelocity;
        real wB = bodyB->angularVelocity;

        ftVector2 dv = (vB + pointConstraint->r2.invCross(wB) - vA - pointConstraint->r1.invCross(wA));
        real jnV = dv.dot(constraint->normal);
        pointConstraint->restitutionBias = -restitution * jnV;


    }
}

void ftContactSolver::init(const ftContactSolverOption &option){
    this->m_option = option;
}

void ftContactSolver::shutdown() {
    //do nothing
}

void ftContactSolver::warmStart() {
    for (uint32 i = 0;i<m_constraintGroup.numConstraint;++i) {
        ftContactConstraint *constraint = &(m_constraintGroup.constraints[i]);

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

            m_constraintGroup.velocities[bodyIDA] -= constraint->invMassA * impulse;
            m_constraintGroup.velocities[bodyIDB] += constraint->invMassB * impulse;

            ftVector2 r1 = pointConstraint->r1;
            ftVector2 r2 = pointConstraint->r2;
            m_constraintGroup.angularVelocities[bodyIDA] -= constraint->invMomentA * r1.cross(impulse);
            m_constraintGroup.angularVelocities[bodyIDB] += constraint->invMomentB * r2.cross(impulse);
        }
    }
}

void ftContactSolver::solve(real dt) {
    uint8 numIteration = m_option.numIteration;
    while (numIteration > 0) {

        for (uint32 i = 0; i < m_constraintGroup.numConstraint; ++i) {

            ftContactConstraint *constraint = &(m_constraintGroup.constraints[i]);
            uint32 bodyIDA = constraint->bodyIDA;
            uint32 bodyIDB = constraint->bodyIDB;
            ftVector2 normal = constraint->normal;
            ftVector2 tangent = normal.tangent();

            for (uint8 j = 0; j<constraint->numContactPoint; ++j) {

                ftContactPointConstraint *pointConstraint = &(constraint->pointConstraint[j]);

                //normal impulse
                {
                    ftVector2 vA = m_constraintGroup.velocities[bodyIDA];
                    ftVector2 vB = m_constraintGroup.velocities[bodyIDB];
                    real wA = m_constraintGroup.angularVelocities[bodyIDA];
                    real wB = m_constraintGroup.angularVelocities[bodyIDB];

                    ftVector2 dv = (vB + pointConstraint->r2.invCross(wB) - vA - pointConstraint->r1.invCross(wA));
                    real jnV = dv.dot(normal);
                    real nLambda = (-jnV + pointConstraint->positionBias/dt + pointConstraint->restitutionBias) * pointConstraint->normalMass;

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

                    m_constraintGroup.velocities[bodyIDA] -= constraint->invMassA * nLinearI;
                    m_constraintGroup.velocities[bodyIDB] += constraint->invMassB * nLinearI;

                    m_constraintGroup.angularVelocities[bodyIDA] -= constraint->invMomentA * nAngularIA;
                    m_constraintGroup.angularVelocities[bodyIDB] += constraint->invMomentB * nAngularIB;

                }

                //tangent impulse
                {
                    ftVector2 vA = m_constraintGroup.velocities[bodyIDA];
                    ftVector2 vB = m_constraintGroup.velocities[bodyIDB];
                    real wA = m_constraintGroup.angularVelocities[bodyIDA];
                    real wB = m_constraintGroup.angularVelocities[bodyIDB];

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

                    m_constraintGroup.velocities[bodyIDA] -= constraint->invMassA * nLinearI;
                    m_constraintGroup.velocities[bodyIDB] += constraint->invMassB * nLinearI;

                    m_constraintGroup.angularVelocities[bodyIDA] -= constraint->invMomentA * nAngularIA;
                    m_constraintGroup.angularVelocities[bodyIDB] += constraint->invMomentB * nAngularIB;

                }

            }

        }
        numIteration --;
    }

    for (uint32 i = 0;i < m_constraintGroup.numBody; i++) {
        m_constraintGroup.bodies[i]->velocity = m_constraintGroup.velocities[i];
        m_constraintGroup.bodies[i]->angularVelocity = m_constraintGroup.angularVelocities[i];
    }

    for (uint32 i = 0; i < m_constraintGroup.numConstraint; ++i) {
        ftContact* contact = m_constraintGroup.constraints[i].contact;
        ftManifold* manifold = &(contact->manifold);
        ftContactConstraint* constraint = &(m_constraintGroup.constraints[i]);
        for (uint32 j = 0; j < contact->manifold.numContact; ++j) {
            manifold->contactPoints[j].nIAcc =
                    constraint->pointConstraint[j].nIAcc;
            manifold->contactPoints[j].tIAcc =
                    constraint->pointConstraint[j].tIAcc;
        }
    }
}

void ftContactSolver::createConstraints(const ftIsland& island) {

    int nBody = island.bodies.getSize();
    int nContact = island.contacts.getSize();

    m_constraintGroup.positions = new ftVector2[nBody];
    m_constraintGroup.velocities = new ftVector2[nBody];
    m_constraintGroup.angularVelocities = new real[nBody];
    m_constraintGroup.bodies = new ftBody*[nBody];
    m_constraintGroup.constraints = new ftContactConstraint[nContact];

    for (int i=0;i<nBody;i++) {
        ftBody* body = island.bodies[i];
        ftVector2 haha = body->transform.center;
        m_constraintGroup.positions[i] = haha;
        m_constraintGroup.velocities[i] = island.bodies[i]->velocity;
        m_constraintGroup.angularVelocities[i] = island.bodies[i]->angularVelocity;
        m_constraintGroup.bodies[i] = island.bodies[i];
    }

    for (int i=0;i<nContact;i++) {
        ftContact* contact = island.contacts[i];

        ftCollider* colliderA = (ftCollider*) contact->userdataA;
        ftCollider* colliderB = (ftCollider*) contact->userdataB;
        createContactConstraint(colliderA, colliderB, contact, &(m_constraintGroup.constraints[i]));

    }

    m_constraintGroup.numConstraint = nContact;
    m_constraintGroup.numBody = nBody;

}

void ftContactSolver::clearConstraints() {
    delete[] m_constraintGroup.positions;
    delete[] m_constraintGroup.velocities;
    delete[] m_constraintGroup.angularVelocities;
    delete[] m_constraintGroup.constraints;
}
