//
// Created by Kevin Yu on 2016-06-26.
//

#ifndef FALTON_FTPISTONJOINT_H
#define FALTON_FTPISTONJOINT_H


#include <falton/physics/joint/ftJoint.h>

class ftPistonJoint : public ftJoint {
public:

    static ftPistonJoint* create (ftBody* bodyA, ftBody* bodyB, ftVector2 axis, ftVector2 lAnchorA, ftVector2 lAnchorB);

    void preSolve(real dt) override;
    void warmStart(ftVector2* vArray, real* wArray) override;
    void solve(real dt, ftVector2* vArray, real* wArray) override;

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

};


#endif //FALTON_FTPISTONJOINT_H
