//
// Created by Kevin Yu on 12/30/15.
//

#ifndef FALTON_FTNSQUAREDBROADPHASE_H
#define FALTON_FTNSQUAREDBROADPHASE_H

#include<queue>

#include <falton/physics/collision/broadphase/ftBroadphaseSystem.h>
#include <falton/container/ftIntQueue.h>
#include <falton/container/ftChunkArray.h>
#include <falton/physics/shape/ftAABB.h>

struct ftCollisionShape;

class ftNSquaredBroadphase : public ftBroadphaseSystem{
public:

    ftNSquaredBroadphase() {};
    ~ftNSquaredBroadphase() {};

    void init();
    void shutdown();

    ftBroadphaseHandle addShape(const ftCollisionShape* const colShape, const void* const userData);
    void removeShape(ftBroadphaseHandle handle);
    void moveShape(ftBroadphaseHandle handle);
    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs);

private:

    struct ftElem {
        ftAABB aabb;
        const ftCollisionShape* collisionShape;
        const void* userdata;

    };

    uint32 nShape;
    ftChunkArray<ftElem> *elements;
    ftIntQueue freeHandleList;

};


#endif //FALTON_FTNSQUAREDBROADPHASE_H
