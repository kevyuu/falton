//
// Created by Kevin Yu on 4/13/16.
//

#include <falton/physics/dynamic/ftBody.h>
#include "falton/physics/joint/ftHingeJoint.h"
#include <iostream>

using namespace std;

ftHingeJoint* ftHingeJoint::create(ftBody *bodyA, ftBody *bodyB, ftVector2 anchorPoint) {

    ftHingeJoint *joint = new ftHingeJoint;
    joint->bodyA = bodyA;
    joint->bodyB = bodyB;
    joint->anchorPoint = anchorPoint;

    joint->localAnchorA = bodyA->transform.rotation.invRotate(anchorPoint - bodyA->transform.center);
    joint->localAnchorB = bodyB->transform.rotation.invRotate(anchorPoint - bodyB->transform.center);
    joint->torqueFriction = 0;

    return joint;

}

void ftHingeJoint::preSolve(real dt) {

    bodyIDA = bodyA->islandId;
    bodyIDB = bodyB->islandId;

    invMassA = bodyA->inverseMass;
    invMomentA = bodyA->inverseMoment;
    invMassB = bodyB->inverseMass;
    invMomentB = bodyB->inverseMoment;

    rA = bodyA->transform.rotation * (localAnchorA - bodyA->centerOfMass);
    rB = bodyB->transform.rotation * (localAnchorB - bodyB->centerOfMass);

    invKDistance.element[0][0] = invMassA + invMassB +
            invMomentA * rA.y * rA.y + invMomentB * rB.y * rB.y;
    invKDistance.element[0][1] = -invMassA * rA.y * rA.x - invMassB * rB.y * rB.x;
    invKDistance.element[1][0] = invKDistance.element[0][1];
    invKDistance.element[1][1] = invMassA + invMassB +
            invMomentA * rA.x * rA.x + invMomentB * rB.x * rB.x;

    invKDistance.invert();

    fIAcc = 0;
    fMaxImpulse = torqueFriction * dt;
    invKFriction = 1 / (invMomentA + invMomentB);

}

void ftHingeJoint::warmStart(ftVector2* vArray, real* wArray) {
    //distance constraint
    {
        ftVector2 impulse = dIAcc;
        vArray[bodyIDA] -= (invMassA * impulse);
        wArray[bodyIDA] -= (invMomentA * rA.cross(impulse));

        vArray[bodyIDB] += (invMassB * impulse);
        wArray[bodyIDB] += (invMomentB * rB.cross(impulse));
    }

    //friction constraint
    {
        real impulse = fIAcc;

        wArray[bodyIDB] += invMomentB * impulse;
        wArray[bodyIDA] -= invMomentA * impulse;
    }
}

void ftHingeJoint::solve(real dt, ftVector2* vArray, real* wArray) {

    //distance constraint
    {
        ftVector2 jv = vArray[bodyIDB] + rB.invCross(wArray[bodyIDB])
                       - vArray[bodyIDA] - rA.invCross(wArray[bodyIDA]);

        ftVector2 impulse = invKDistance * (jv * -1);
        dIAcc += impulse;

        vArray[bodyIDA] -= (invMassA * impulse);
        wArray[bodyIDA] -= (invMomentA * rA.cross(impulse));

        vArray[bodyIDB] += (invMassB * impulse);
        wArray[bodyIDB] += (invMomentB * rB.cross(impulse));
    }

    //friction constraint
    {
        real jv = wArray[bodyIDB] - wArray[bodyIDA];
        real impulse = invKFriction * (jv * -1);
        real oldI = fIAcc;
        fIAcc += impulse;
        if (fIAcc > fMaxImpulse) {
            fIAcc = fMaxImpulse;
        } else if (fIAcc < -fMaxImpulse) {
            fIAcc = -fMaxImpulse;
        }
        impulse = fIAcc - oldI;

        wArray[bodyIDB] += invMomentB * impulse;
        wArray[bodyIDA] -= invMomentA * impulse;

    }

}