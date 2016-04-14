//
// Created by Kevin Yu on 4/13/16.
//

#include <falton/physics/ftBody.h>
#include "falton/physics/Joint/ftPinJoint.h"

ftPinJoint* ftPinJoint::create(ftBody *bodyA, ftBody *bodyB, ftVector2 anchorPoint) {

    ftPinJoint *joint = new ftPinJoint;
    joint->bodyA = bodyA;
    joint->bodyB = bodyB;
    joint->anchorPoint = anchorPoint;

    return joint;

}

void ftPinJoint::preSolve(real dt) {
    r1 = anchorPoint - bodyA->transform.center;
    r2 = anchorPoint - bodyB->transform.center;

    invK.element[0][0] = bodyA->inverseMass + bodyB->inverseMass +
            bodyA->inverseMoment * r1.y * r1.y + bodyB->inverseMoment * r2.y * r2.y;
    invK.element[0][1] = -bodyA->inverseMass * r1.y * r1.x - bodyB->inverseMass * r2.y * r2.x;
    invK.element[1][0] = invK.element[0][1];
    invK.element[1][1] = bodyA->inverseMass + bodyB->inverseMass +
            bodyA->inverseMoment * r1.x * r1.x + bodyB->inverseMoment * r2.x * r2.x;

    invK.invert();

}

void ftPinJoint::solve() {

    ftVector2 jv = bodyB->velocity + r2.invCross(bodyB->angularVelocity) - bodyA->velocity - r1.invCross(bodyA->angularVelocity);

    ftVector2 impulse = invK * (jv * -1);

    bodyA->velocity -= (bodyA->inverseMass * impulse);
    bodyA->angularVelocity -= (bodyA->inverseMoment * r1.cross(impulse));

    bodyB->velocity += (bodyB->inverseMass * impulse);
    bodyB->angularVelocity += (bodyB->inverseMoment * r2.cross(impulse));
}