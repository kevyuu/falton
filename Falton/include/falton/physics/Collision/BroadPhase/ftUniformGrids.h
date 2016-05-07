//
// Created by Kevin Yu on 5/4/16.
//

#ifndef FALTON_FTUNIFORMGRIDS_H
#define FALTON_FTUNIFORMGRIDS_H


#include <falton/container/ftIntQueue.h>
#include "falton/physics/collision/broadphase/ftBroadphaseSystem.h"

class ftUniformGrids : public ftBroadphaseSystem{
public:

    struct ftConfig {
        uint32 cellWidth;
        uint32 cellHeight;
        uint32 worldWidth;
        uint32 worldHeight;
    };

    void setConfig(ftConfig config);

    void init() override;
    void shutdown() override;

    ftBroadphaseHandle addShape(const ftCollisionShape *const colShape, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle) override;

    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) override;

private:

    struct ftProxy {
        const ftCollisionShape* collisionShape;
        const void* userdata;
    };

    struct ftElem {
        ftProxy* proxy;
        ftElem* next;
    };

    ftConfig config;

    uint32 nRow;
    uint32 nCol;

    ftChunkArray<ftProxy> *proxies;

    ftIntQueue freeHandles;

    ftElem** grids;


};


#endif //FALTON_FTUNIFORMGRIDS_H
