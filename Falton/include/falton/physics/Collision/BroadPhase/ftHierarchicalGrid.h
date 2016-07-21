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
        uint32 nLevel = 32;
        real baseSize = 1.5;
        real sizeMul = 2;
        uint32 nBucket = 1024;
    };

    int getMemoryUsage() override;

    ftHierarchicalGrid(){};

    void setConfiguration(ftConfig config);

    void init() override;
    void shutdown() override;

    ftBroadphaseHandle addShape(const ftShape* shape, const ftTransform& transform, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform) override;

    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) override;

    void regionQuery(const ftAABB& region, ftChunkArray<const void*>* results) override;


private:

    struct ftElem {
        const void* userdata;
        ftAABB aabb;
        uint32 level;
        uint32 bucketIndex;
        ftElem* prev;
        union {
            ftElem* next;
            uint32 nextFree;
        };
    };

    static const uint32 NULL_ELEM = nulluint;

    uint32* m_nObject;

    uint32 m_nLevel = 32;
    real m_baseSize = 5;
    real m_sizeMul = 2;

    real* m_cellSizeTable;

    ftChunkArray<ftElem> m_elemList;
    uint32 m_freeElem;

    ftElem** m_elemBucket;
    uint32 m_bucketCapacity;

    uint32 computeHashIndex(int32 x, int32 y, uint32 level);
    void insertElemToBucket(ftElem* elem, uint32 bucketIndex);
    void addCollidingPairInBucket(ftElem *elem, uint32 bucketIndex, ftChunkArray<ftBroadPhasePair> *pairs);
    void unlink(ftElem* elem);
};


#endif //FALTON_FTHIERARCHICALGRID_H
