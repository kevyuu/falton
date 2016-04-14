//
// Created by Kevin Yu on 12/29/15.
//

#ifndef FALTON_NARROWPHASE_H
#define FALTON_NARROWPHASE_H

#include "falton/physics/shape/ftShape.h"
#include "falton/physics/Collision/ftCollisionSystem.h"

struct ftBroadPhasePair {
    ColHandle handleA;
    ColHandle handleB;
};

class ftBroadPhase {
public:

    ftBroadPhase() {

    };

    virtual void init(ftTransformShape *collisionShapes) = 0;
    virtual void shutdown() = 0;

    virtual void newShape(ColHandle handle) = 0;
    virtual void removeShape(ColHandle handle) = 0;
    virtual void moveShape(ColHandle handle) = 0;
    virtual void findPairs(ftChunkArray<ftBroadPhasePair> &pairs) = 0;

protected:
    ftTransformShape *collisionShapes;

};


#endif //FALTON_NARROWPHASE_H
