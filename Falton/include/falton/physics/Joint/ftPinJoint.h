//
// Created by Kevin Yu on 4/13/16.
//

#ifndef FALTON_FTPINJOINT_H
#define FALTON_FTPINJOINT_H
#include "falton/math/math.h"

struct ftBody;

class ftPinJoint {
    ftBody* bodyA;
    ftBody* bodyB;
    ftVector2 anchorPoint;

    ftMat2x2 invK;
    ftVector2 r1;
    ftVector2 r2;

public:

    static ftPinJoint* create(ftBody* bodyA, ftBody* bodyB, ftVector2 anchorPoint);
    void preSolve(real dt);
    void solve();

};


#endif //FALTON_FTPINJOINT_H
