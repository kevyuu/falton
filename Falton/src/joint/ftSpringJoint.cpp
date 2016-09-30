//
// Created by Kevin Yu on 2016-06-26.
//

#include <falton/dynamic/ftBody.h>
#include <falton/joint/ftSpringJoint.h>

ftSpringJoint* ftSpringJoint::create(ftBody *bodyA, ftBody *bodyB, ftVector2 lAnchorA, ftVector2 lAnchorB) {
    ftSpringJoint* joint = new ftSpringJoint;

    joint->bodyA = bodyA;
    joint->bodyB = bodyB;

    joint->localAnchorA = lAnchorA;
    joint->localAnchorB = lAnchorB;

    ftVector2 anchorDiff = bodyB->transform * lAnchorB - bodyA->transform * lAnchorA;
    joint->springAxis = anchorDiff.unit();
    joint->tAxis = joint->springAxis.perpendicular();

    joint->stiffness = 1;
    joint->restLength = anchorDiff.magnitude();

    return joint;
}

void ftSpringJoint::preSolve(real dt) {
    bodyIDA = bodyA->islandId;
    bodyIDB = bodyB->islandId;

    invMassA = bodyA->inverseMass;
    invMomentA = bodyA->inverseMoment;
    invMassB = bodyB->inverseMass;
    invMomentB = bodyB->inverseMoment;

    invKRot = 1 / (invMomentA + invMomentB);

    rA = bodyA->transform.rotation * (localAnchorA - bodyA->centerOfMass);
    rB = bodyB->transform.rotation * (localAnchorB - bodyB->centerOfMass);
    real kTrans = invMassA + invMassB;
    real rtA = rA.cross(tAxis);
    real rtB = rB.cross(tAxis);
    kTrans += (invMomentA * rtA * rtA + invMomentB * rtB * rtB);
    invKTrans = 1/kTrans;

    //apply spring force
    {
        ftVector2 worldAnchorA = bodyA->transform * localAnchorA;
        ftVector2 worldAnchorB = bodyB->transform * localAnchorB;
        real dist = (worldAnchorB - worldAnchorA).dot(springAxis);
        real forceMagnitude = (restLength - dist) * stiffness;
        real impulseMagnitude = forceMagnitude * dt;
        ftVector2 impulse = springAxis * impulseMagnitude;
        bodyB->velocity += invMassB * impulse;
        bodyA->velocity -= invMassA * impulse;
    }
}

void ftSpringJoint::warmStart(ftVector2* vArray, real* wArray) {
    //translation constraint
    {
        ftVector2 linearI = tAxis * translationIAcc;
        real angularIA = rA.cross(tAxis) * translationIAcc;
        real angularIB = rB.cross(tAxis) * translationIAcc;

        vArray[bodyIDA] -= (invMassA * linearI);
        wArray[bodyIDA] -= (invMomentA * angularIA);

        vArray[bodyIDB] += (invMassB * linearI);
        wArray[bodyIDB] += (invMomentB * angularIB);
    }

    //rotation constraint
    {
        wArray[bodyIDB] += invMomentB * rotationIAcc;
        wArray[bodyIDA] -= invMomentA * rotationIAcc;
    }
}

void ftSpringJoint::solve(real dt, ftVector2 *vArray, real *wArray) {

    //translation constraint
    {
        ftVector2 dv = vArray[bodyIDB] + rB.invCross(wArray[bodyIDB])
                       - vArray[bodyIDA] - rA.invCross(wArray[bodyIDA]);
        real jv = dv.dot(tAxis);

        real impulse = invKTrans * (jv * -1);

        translationIAcc += impulse;

        ftVector2 linearI = tAxis * impulse;
        real angularIA = rA.cross(tAxis) * impulse;
        real angularIB = rB.cross(tAxis) * impulse;

        vArray[bodyIDA] -= (invMassA * linearI);
        wArray[bodyIDA] -= (invMomentA * angularIA);

        vArray[bodyIDB] += (invMassB * linearI);
        wArray[bodyIDB] += (invMomentB * angularIB);

    }

    //rotation constraint
    {
        real jv = wArray[bodyIDB] - wArray[bodyIDA];
        real impulse = invKRot * (jv * -1);

        rotationIAcc += impulse;

        wArray[bodyIDB] += invMomentB * impulse;
        wArray[bodyIDA] -= invMomentA * impulse;
    }

}