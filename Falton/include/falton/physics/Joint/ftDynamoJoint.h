//
// Created by Kevin Yu on 2016-07-08.
//

#ifndef FALTON_FTDYNAMOJOINT_H
#define FALTON_FTDYNAMOJOINT_H


#include "ftJoint.h"

class ftDynamoJoint : public ftJoint{

public:

    static ftDynamoJoint* create(ftBody* bodyA, ftBody* bodyB, real targetRate, real maxTorque);

    void preSolve(real dt) override;
    void warmStart(ftVector2* vArray, real* wArray) override;
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

};


#endif //FALTON_FTDYNAMOJOINT_H
