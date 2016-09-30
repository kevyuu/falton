//
// Created by Kevin Yu on 2016-07-08.
//

#include <falton/physics/dynamic/ftBody.h>
#include "falton/physics/joint/ftDynamoJoint.h"

ftDynamoJoint* ftDynamoJoint::create(ftBody* bodyA, ftBody* bodyB, real targetRate, real maxTorque) {
    ftDynamoJoint* joint = new ftDynamoJoint;
    joint->bodyA = bodyA;
    joint->bodyB = bodyB;

    joint->targetRate = targetRate;
    joint->maxTorque = maxTorque;

    return joint;
}

void ftDynamoJoint::preSolve(real dt) {

    invMomentA = bodyA->inverseMoment;
    invMomentB = bodyB->inverseMoment;
    invK = 1 / (invMomentA + invMomentB);

    bodyIDA = bodyA->islandId;
    bodyIDB = bodyB->islandId;
    maxImpulse = maxTorque * dt;

}

void ftDynamoJoint::warmStart(ftVector2* vArray, real* wArray) {
    wArray[bodyIDB] += invMomentB * iAcc;
    wArray[bodyIDA] -= invMomentA * iAcc;
}

void ftDynamoJoint::solve(real dt, ftVector2 *vArray, real *wArray) {

    real jV = wArray[bodyIDB] - wArray[bodyIDA] - targetRate;

    real impulse = invK * -jV;

    real oldI = iAcc;
    iAcc += impulse;
    if (iAcc > maxImpulse) {
        iAcc = maxImpulse;
    }
    impulse = iAcc - oldI;

    wArray[bodyIDB] += invMomentB * impulse;
    wArray[bodyIDA] -= invMomentA * impulse;

}

