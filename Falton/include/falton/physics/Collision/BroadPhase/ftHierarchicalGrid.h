//
// Created by Kevin Yu on 5/5/16.
//

#ifndef FALTON_FTHIERARCHICALGRID_H
#define FALTON_FTHIERARCHICALGRID_H


#include <falton/container/ftBitSet.h>
#include <falton/container/ftIntQueue.h>
#include "ftBroadphaseSystem.h"

class ftHierarchicalGrid : public ftBroadphaseSystem {
public:
    struct ftConfig {
        uint32 nLevel;
        real baseSize;
        real sizeMul;
    };

    ftHierarchicalGrid();

    void setConfig(ftConfig config);

    void init() override;
    void shutdown() override;

    ftBroadphaseHandle addShape(const ftCollisionShape *const colShape, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle) override;

    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) override;


private:

    struct ftElem {
        const ftCollisionShape* collisionShape;
        const void* userdata;
        ftAABB aabb;
        uint32 level;
        uint32 bucketIndex;
        ftElem* prev;
        ftElem* next;
    };

    uint32* m_nObject;

    uint32 m_nLevel;
    real m_baseSize;
    real m_sizeMul;

    real* m_cellSizeTable;

    ftChunkArray<ftElem> m_elemList;
    ftIntQueue freeHandles;

    ftElem** m_elemBucket;
    uint32 m_bucketCapacity;

    uint32 computeHashIndex(int32 x, int32 y, uint32 level);
    void insertElemToBucket(ftElem* elem, uint32 bucketIndex);
    void addCollidingPairInBucket(ftElem *elem, uint32 bucketIndex, ftChunkArray<ftBroadPhasePair> *pairs);
    void unlink(ftElem* elem);
};


#endif //FALTON_FTHIERARCHICALGRID_H
