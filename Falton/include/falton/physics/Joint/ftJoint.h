//
// Created by Kevin Yu on 2016-06-01.
//

#ifndef FALTON_FTJOINT_H
#define FALTON_FTJOINT_H

#include <falton/setting/general.h>
#include <falton/math/ftVector2.h>

struct ftBody;

struct ftJoint {
    ftBody* bodyA;
    ftBody* bodyB;

    int32 islandIndex;

    virtual void preSolve(real dt) = 0;
    virtual void solve(real dt, ftVector2* linearVelocities, real* angularVelocities) = 0;
};

struct ftJointEdge {
    ftBody* other;
    ftJoint* joint;
    ftJointEdge* prev;
    ftJointEdge* next;
};

#endif //FALTON_FTJOINT_H
