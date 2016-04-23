//
// Created by Kevin Yu on 12/30/15.
//

#ifndef FALTON_FTNSQUAREDBROADPHASE_H
#define FALTON_FTNSQUAREDBROADPHASE_H

#include<queue>

#include "ftBroadPhase.h"
#include "falton/container/ftIntQueue.h"
#include "falton/container/ftChunkArray.h"
#include "falton/physics/shape/ftAABB.h"

class ftNSquaredBroadphase : public ftBroadPhase{
public:

    ftNSquaredBroadphase() : aabbBuffers(128) , lastHandle(0){ };
    ~ftNSquaredBroadphase() {};

    void init(ftTransformShape* collisionShapes);
    void shutdown();

    void newShape(ColHandle handle);
    void removeShape(ColHandle handle);
    void moveShape(ColHandle handle);
    void findPairs(ftChunkArray<ftBroadPhasePair> &pairs);

private:

    ftChunkArray<ftAABB> aabbBuffers;
    ftIntQueue dirtyHandles;

    uint32 lastHandle;

};


#endif //FALTON_FTNSQUAREDBROADPHASE_H
