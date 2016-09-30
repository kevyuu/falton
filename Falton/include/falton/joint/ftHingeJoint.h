//
// Created by Kevin Yu on 4/13/16.
//

#ifndef FALTON_FTPINJOINT_H
#define FALTON_FTPINJOINT_H

#include <falton/joint/ftJoint.h>

struct ftBody;

class ftHingeJoint : public ftJoint {
public:
    real torqueFriction;
    static ftHingeJoint* create(ftBody* bodyA, ftBody* bodyB, ftVector2 anchorPoint);
    void preSolve(real dt);
    void warmStart(ftVector2* vArray, real* wArray);
    void solve(real dt, ftVector2* vArray, real* wArray);

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

};


#endif //FALTON_FTPINJOINT_H
