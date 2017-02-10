#include "falton/dynamic/ftJoint.h"

ftDistanceJoint *ftDistanceJoint::create(ftBody *bodyA,
                                         ftBody *bodyB,
                                         ftVector2 localAnchorA,
                                         ftVector2 localAnchorB)
{
    ftDistanceJoint *joint = new ftDistanceJoint;
    joint->jointType = ftJoint::DISTANCE_JOINT;
    joint->bodyA = bodyA;
    joint->bodyB = bodyB;
    joint->localAnchorA = localAnchorA;
    joint->localAnchorB = localAnchorB;
    joint->distance = (bodyA->transform * localAnchorA - bodyB->transform * localAnchorB).magnitude();
    joint->iAcc = 0;

    return joint;
}

ftDynamoJoint *ftDynamoJoint::create(ftBody *bodyA,
                                     ftBody *bodyB,
                                     real targetRate,
                                     real maxTorque)
{
    ftDynamoJoint *joint = new ftDynamoJoint;
    joint->bodyA = bodyA;
    joint->bodyB = bodyB;
    joint->targetRate = targetRate;
    joint->maxTorque = maxTorque;
    return joint;
}

ftHingeJoint *ftHingeJoint::create(ftBody *bodyA,
                                   ftBody *bodyB,
                                   ftVector2 anchorPoint)
{

    ftHingeJoint *joint = new ftHingeJoint;
    joint->jointType = ftJoint::HINGE_JOINT;
    joint->bodyA = bodyA;
    joint->bodyB = bodyB;
    joint->anchorPoint = anchorPoint;

    joint->localAnchorA = bodyA->transform.rotation.invRotate(anchorPoint - bodyA->transform.center);
    joint->localAnchorB = bodyB->transform.rotation.invRotate(anchorPoint - bodyB->transform.center);
    joint->torqueFriction = 0;

    return joint;
}

ftPistonJoint *ftPistonJoint::create(ftBody *bodyA,
                                     ftBody *bodyB,
                                     ftVector2 axis,
                                     ftVector2 lAnchorA,
                                     ftVector2 lAnchorB)
{
    ftPistonJoint *joint = new ftPistonJoint;
    joint->jointType = ftJoint::PISTON_JOINT;
    joint->bodyA = bodyA;
    joint->bodyB = bodyB;
    joint->localAnchorA = lAnchorA;
    joint->localAnchorB = lAnchorB;
    joint->tAxis = axis.perpendicular();

    return joint;
}

ftSpringJoint *ftSpringJoint::create(ftBody *bodyA,
                                     ftBody *bodyB,
                                     ftVector2 lAnchorA,
                                     ftVector2 lAnchorB)
{
    ftSpringJoint *joint = new ftSpringJoint;
    joint->jointType = ftJoint::SPRING_JOINT;
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