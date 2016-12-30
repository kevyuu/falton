#pragma once

#include "falton/setting.h"
#include "falton/math.h"
#include "falton/dynamic/ftBody.h"
#include "ftBody.h"


class ftJoint
{

  public:
    enum ftJointType
    {
        DISTANCE_JOINT,
        DYNAMO_JOINT,
        HINGE_JOINT,
        PISTON_JOINT,
        SPRING_JOINT,
        COUNT_JOINT_TYPE
    };

    ftJointType jointType;

    ftBody *bodyA;
    ftBody *bodyB;

    int32 islandIndex;
};

struct ftJointEdge
{
    ftBody *other;
    ftJoint *joint;
    ftJointEdge *prev;
    ftJointEdge *next;
};

class ftDistanceJoint : public ftJoint
{
  public:
    static ftDistanceJoint *create(ftBody *bodyA,
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

    real distance;

  private:
    ftVector2 localAnchorA;
    ftVector2 localAnchorB;

    //derivative
    int32 bodyIDA, bodyIDB;

    real invK;
    real invMassA, invMassB;
    real invMomentA, invMomentB;
    real positionBias;
    ftVector2 jVa;
    real jWa, jWb;

    real iAcc;

    friend class ftJointSolver;
};

class ftDynamoJoint : public ftJoint
{

  public:
    static ftDynamoJoint *create(ftBody *bodyA,
                                 ftBody *bodyB,
                                 real targetRate,
                                 real maxTorque)
    {
        ftDynamoJoint* joint = new ftDynamoJoint;
        joint->bodyA = bodyA;
        joint->bodyB = bodyB;
        joint->targetRate = targetRate;
        joint->maxTorque = maxTorque;
        return joint;
    }

    real targetRate;
    real maxTorque;
    real maxImpulse;
    real iAcc;

  private:
    int32 bodyIDA, bodyIDB;

    real invK;
    real invMomentA;
    real invMomentB;

    friend class ftPhysicsSystem;
    friend class ftJointSolver;
};

class ftHingeJoint : public ftJoint
{
  public:
    real torqueFriction;
    static ftHingeJoint *create(ftBody *bodyA,
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

  private:
    ftVector2 anchorPoint;

    //derivative variable
    int32 bodyIDA, bodyIDB;

    real invMassA;
    real invMomentA;
    real invMassB;
    real invMomentB;

    //friction constraint
    real fIAcc;
    real fMaxImpulse;
    real invKFriction;

    //distance constraint
    ftMat2x2 invKDistance;
    ftVector2 localAnchorA;
    ftVector2 localAnchorB;
    ftVector2 rA;
    ftVector2 rB;
    ftVector2 dIAcc;

    friend class ftJointSolver;
};

class ftPistonJoint : public ftJoint
{
  public:
    static ftPistonJoint *create(ftBody *bodyA,
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

  private:
    int32 bodyIDA, bodyIDB;

    real invMassA;
    real invMomentA;
    real invMassB;
    real invMomentB;

    real invKRot;

    real invKTrans;
    ftVector2 localAnchorA;
    ftVector2 localAnchorB;
    ftVector2 rA;
    ftVector2 rB;
    ftVector2 tAxis; // axis that perpendicular to translation axis

    friend class ftJointSolver;
};

class ftSpringJoint : public ftJoint
{
  public:
    static ftSpringJoint *create(ftBody *bodyA,
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

    real stiffness;
    real restLength;

  private:
    int32 bodyIDA, bodyIDB;

    real invMassA;
    real invMomentA;
    real invMassB;
    real invMomentB;

    ftVector2 localAnchorA;
    ftVector2 localAnchorB;
    ftVector2 rA;
    ftVector2 rB;
    ftVector2 springAxis; // pointing from bodyA to bodyB;
    ftVector2 tAxis;      // axis that perpendicular to spring axis

    real translationIAcc;
    real rotationIAcc;

    real invKRot;
    real invKTrans;

    friend class ftJointSolver;
};