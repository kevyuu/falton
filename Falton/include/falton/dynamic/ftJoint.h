#pragma once

#include <falton/setting.h>
#include <falton/math.h>

class ftBody;

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

    virtual void preSolve(real dt) = 0;
    virtual void warmStart(ftVector2 *vArray, real *wArray) = 0;
    virtual void solve(real dt, ftVector2 *vArray, real *wArray) = 0;
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
                                   ftVector2 localAnchorB);

    void preSolve(real dt) override;
    void warmStart(ftVector2 *vArray, real *wArray) override;
    void solve(real dt, ftVector2 *vArray, real *wArray) override;

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
    static ftDynamoJoint *create(ftBody *bodyA, ftBody *bodyB, real targetRate, real maxTorque);

    void preSolve(real dt) override;
    void warmStart(ftVector2 *vArray, real *wArray) override;
    void solve(real dt, ftVector2 *vArray, real *wArray) override;

    real targetRate;
    real maxTorque;
    real maxImpulse;
    real iAcc;

  private:
    int32 bodyIDA, bodyIDB;

    real invK;
    real invMomentA;
    real invMomentB;

    friend class ftJointSolver;
};

class ftHingeJoint : public ftJoint
{
  public:
    real torqueFriction;
    static ftHingeJoint *create(ftBody *bodyA, ftBody *bodyB, ftVector2 anchorPoint);
    void preSolve(real dt);
    void warmStart(ftVector2 *vArray, real *wArray);
    void solve(real dt, ftVector2 *vArray, real *wArray);

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
    static ftPistonJoint *create(ftBody *bodyA, ftBody *bodyB, ftVector2 axis, ftVector2 lAnchorA, ftVector2 lAnchorB);

    void preSolve(real dt) override;
    void warmStart(ftVector2 *vArray, real *wArray) override;
    void solve(real dt, ftVector2 *vArray, real *wArray) override;

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
    static ftSpringJoint *create(ftBody *bodyA, ftBody *bodyB, ftVector2 lAnchorA, ftVector2 lAnchorB);

    void preSolve(real dt) override;
    void warmStart(ftVector2 *vArray, real *wArray) override;
    void solve(real dt, ftVector2 *vArray, real *wArray) override;

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