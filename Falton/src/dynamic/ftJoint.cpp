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

    return joint;
}

ftDynamoJoint *ftDynamoJoint::create(ftBody *bodyA,
                                     ftBody *bodyB,
                                     real targetRate,
                                     real maxTorque)
{
    ftDynamoJoint *joint = new ftDynamoJoint;
    joint->jointType = ftJoint::DYNAMO_JOINT;
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

    joint->localAnchorA = bodyA->transform.invTransform(anchorPoint);
    joint->localAnchorB = bodyB->transform.invTransform(anchorPoint);
    joint->torqueFriction = 0;

    joint->enableLimit = false;
    joint->lowerLimit = -2 * 3.14;
    joint->upperLimit = 2 * 3.14;
    joint->limitImpulseAcc = 0;

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
    joint->localAxis = axis;
    joint->refAngle = bodyB->transform.rotation.angle - bodyA->transform.rotation.angle;

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
    joint->stiffness = 1;
    joint->restLength = anchorDiff.magnitude();

	joint->translationIAcc = 0;
	joint->rotationIAcc = 0;

    return joint;
}