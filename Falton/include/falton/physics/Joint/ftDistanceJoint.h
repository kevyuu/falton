//
// Created by Kevin Yu on 2016-06-03.
//

#ifndef FALTON_FTDISTANCEJOINT_H
#define FALTON_FTDISTANCEJOINT_H

#include <falton/math/math.h>
#include <falton/physics/joint/ftJoint.h>

class ftDistanceJoint : public ftJoint{
public:

    static ftDistanceJoint* create(ftBody* bodyA, ftBody* bodyB, ftVector2 localAnchorA, ftVector2 localAnchorB);

    void preSolve(real dt) override;
    void warmStart(ftVector2* vArray, real* wArray) override;
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

};


#endif //FALTON_FTDISTANCEJOINT_H
