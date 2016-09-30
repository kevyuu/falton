//
// Created by Kevin Yu on 2016-06-01.
//

#ifndef FALTON_FTJOINT_H
#define FALTON_FTJOINT_H

#include <falton/setting.h>
#include <falton/math.h>

struct ftBody;

struct ftJoint {
    ftBody* bodyA;
    ftBody* bodyB;

    int32 islandIndex;

    virtual void preSolve(real dt) = 0;
    virtual void warmStart(ftVector2* vArray, real* wArray) = 0;
    virtual void solve(real dt, ftVector2* vArray, real* wArray) = 0;
};

struct ftJointEdge {
    ftBody* other;
    ftJoint* joint;
    ftJointEdge* prev;
    ftJointEdge* next;
};

#endif //FALTON_FTJOINT_H
