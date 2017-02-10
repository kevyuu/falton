#pragma once

#include "falton/setting.h"
#include "falton/math.h"
#include "falton/dynamic/ftBody.h"

struct ftJoint
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

struct ftDistanceJoint : public ftJoint
{
  public:
    static ftDistanceJoint *create(ftBody *bodyA,
                                   ftBody *bodyB,
                                   ftVector2 localAnchorA,
                                   ftVector2 localAnchorB);
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

struct ftDynamoJoint : public ftJoint
{

  public:
    static ftDynamoJoint *create(ftBody *bodyA,
                                 ftBody *bodyB,
                                 real targetRate,
                                 real maxTorque);
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

struct ftHingeJoint : public ftJoint
{
  public:
    real torqueFriction;
    static ftHingeJoint *create(ftBody *bodyA,
                                ftBody *bodyB,
                                ftVector2 anchorPoint);

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

struct ftPistonJoint : public ftJoint
{
  public:
    static ftPistonJoint *create(ftBody *bodyA,
                                 ftBody *bodyB,
                                 ftVector2 axis,
                                 ftVector2 lAnchorA,
                                 ftVector2 lAnchorB);

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

struct ftSpringJoint : public ftJoint
{
  public:
    static ftSpringJoint *create(ftBody *bodyA,
                                 ftBody *bodyB,
                                 ftVector2 lAnchorA,
                                 ftVector2 lAnchorB);

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