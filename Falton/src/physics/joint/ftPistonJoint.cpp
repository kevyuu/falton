//
// Created by Kevin Yu on 2016-06-26.
//

#include <falton/physics/dynamic/ftBody.h>
#include "falton/physics/joint/ftPistonJoint.h"

ftPistonJoint* ftPistonJoint::create(ftBody *bodyA, ftBody *bodyB, ftVector2 axis, ftVector2 lAnchorA,
                                     ftVector2 lAnchorB) {
    ftPistonJoint* joint = new ftPistonJoint;
    joint->bodyA = bodyA;
    joint->bodyB = bodyB;
    joint->localAnchorA = lAnchorA;
    joint->localAnchorB = lAnchorB;
    joint->tAxis = axis.perpendicular();

}

void ftPistonJoint::preSolve(real dt) {
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

}

void ftPistonJoint::warmStart(ftVector2 *vArray, real *wArray) {
    //TODO
}

void ftPistonJoint::solve(real dt, ftVector2 *vArray, real *wArray) {

    //translation constraint
    {
        ftVector2 jv = vArray[bodyIDB] + rB.invCross(wArray[bodyIDB])
                    - vArray[bodyIDA] - rA.invCross(wArray[bodyIDB]);

        ftVector2 impulse = invKTrans * (jv * -1);

        vArray[bodyIDA] -= (invMassA * impulse);
        wArray[bodyIDA] -= (invMomentA * rA.cross(impulse));

        vArray[bodyIDB] += (invMassB * impulse);
        wArray[bodyIDB] += (invMomentB * rB.cross(impulse));

    }

    //rotation constraint
    {
        real jv = wArray[bodyIDB] - wArray[bodyIDA];
        real impulse = invKRot * (jv * -1);

        wArray[bodyIDB] += invMomentB * impulse;
        wArray[bodyIDA] -= invMomentA * impulse;
    }
}

