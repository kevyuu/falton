//
// Created by Kevin Yu on 2016-06-03.
//

#include <falton/physics/dynamic/ftBody.h>
#include <falton/physics/joint/ftDistanceJoint.h>

#include <iostream>
using namespace std;

ftDistanceJoint* ftDistanceJoint::create(ftBody *bodyA, ftBody *bodyB,
                                         ftVector2 localAnchorA,
                                         ftVector2 localAnchorB) {

    ftDistanceJoint* joint = new ftDistanceJoint;

    joint->bodyA = bodyA;
    joint->bodyB = bodyB;
    joint->localAnchorA = localAnchorA;
    joint->localAnchorB = localAnchorB;
    joint->distance = (bodyA->transform * localAnchorA - bodyB->transform * localAnchorB).magnitude();
    joint->iAcc = 0;

    return joint;

}

void ftDistanceJoint::preSolve(real dt) {

    jVa = (bodyA->transform * localAnchorA - bodyB->transform * localAnchorB);

    bodyIDA = bodyA->islandId;
    invMassA = bodyA->inverseMass;
    invMomentA = bodyA->inverseMoment;
    ftVector2 rA = bodyA->transform.rotation * (localAnchorA - bodyA->centerOfMass);

    bodyIDB = bodyB->islandId;
    invMassB = bodyB->inverseMass;
    invMomentB = bodyB->inverseMoment;
    ftVector2 rB = bodyB->transform.rotation * (localAnchorB - bodyB->centerOfMass);

    jWa = rA.cross(jVa);
    jWb = rB.cross(jVa);

    real squareMagnitude = jVa.square_magnitude();
    real k = squareMagnitude * invMassA + squareMagnitude * invMassB +
            invMomentA * jWa * jWa + invMomentB * jWb * jWb;
    invK = 1 / k;

    real curDistance = jVa.magnitude();
    positionBias = 0.2 * (curDistance - distance) / dt;

}

void ftDistanceJoint::warmStart(ftVector2* vArray, real* wArray) {
    ftVector2 linearI = jVa * iAcc;
    real angularIA = jWa * iAcc;
    real angularIB = jWb * iAcc;

    vArray[bodyIDA] += invMassA * linearI;
    vArray[bodyIDB] -= invMassB * linearI;

    wArray[bodyIDA] += invMomentA * angularIA;
    wArray[bodyIDB] -= invMomentB * angularIB;
}

void ftDistanceJoint::solve(real dt __attribute__((unused)), ftVector2 *vArray, real *wArray) {

    ftVector2 vA = vArray[bodyIDA];
    ftVector2 vB = vArray[bodyIDB];
    real wA = wArray[bodyIDA];
    real wB = wArray[bodyIDB];

    real jv = jVa.dot(vA) - jVa.dot(vB) + jWa * wA - jWb * wB;

    real impulse = invK * (-jv - positionBias);
    iAcc += impulse;

    ftVector2 linearI = jVa * impulse;
    real angularIA = jWa * impulse;
    real angularIB = jWb * impulse;

    vArray[bodyIDA] += invMassA * linearI;
    vArray[bodyIDB] -= invMassB * linearI;

    wArray[bodyIDA] += invMomentA * angularIA;
    wArray[bodyIDB] -= invMomentB * angularIB;

}

