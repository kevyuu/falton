//
// Created by Kevin Yu on 12/29/15.
//

#ifndef FALTON_BROADPHASE_H
#define FALTON_BROADPHASE_H

#include "falton/physics/shape/ftShape.h"

typedef uint32 ftBroadphaseHandle;

struct ftCollisionShape;

struct ftBroadPhasePair {
    const void* userdataA;
    const void* userdataB;
};

class ftBroadphaseSystem {
public:

    ftBroadphaseSystem() {}
    virtual ~ftBroadphaseSystem() {}

    virtual void init() = 0;
    virtual void shutdown() = 0;
    virtual ftBroadphaseHandle addShape(const ftCollisionShape* const colShape, const void* const userData) = 0;
    virtual void removeShape(ftBroadphaseHandle handle) = 0;
    virtual void moveShape(ftBroadphaseHandle handle, const ftCollisionShape& collisionShape) = 0;

    virtual void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) = 0;

protected:

};


#endif //FALTON_NARROWPHASE_H
