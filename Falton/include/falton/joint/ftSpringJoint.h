//
// Created by Kevin Yu on 2016-06-26.
//

#ifndef FALTON_FTSPRINGJOINT_H
#define FALTON_FTSPRINGJOINT_H

#include <falton/joint/ftJoint.h>

class ftSpringJoint : public ftJoint {
public:

    static ftSpringJoint* create (ftBody* bodyA, ftBody* bodyB, ftVector2 lAnchorA, ftVector2 lAnchorB);

    void preSolve(real dt) override;
    void warmStart(ftVector2* vArray, real* wArray) override;
    void solve(real dt, ftVector2* vArray, real* wArray) override;

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
    ftVector2 tAxis; // axis that perpendicular to spring axis

    real translationIAcc;
    real rotationIAcc;

    real invKRot;
    real invKTrans;

};


#endif //FALTON_FTSPRINGJOINT_H
