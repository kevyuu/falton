//
// Created by Kevin Yu on 2016-05-21.
//

#ifndef FALTON_FTTOROIDALGRID_H
#define FALTON_FTTOROIDALGRID_H

#include <falton/physics/collision/broadphase/ftBroadphaseSystem.h>
#include <falton/container/ftBitSet.h>

class ftToroidalGrid : public ftBroadphaseSystem {

public:

    struct ftConfig {
        real cellSize = 1.5;
        uint32 nRow = 32;
        uint32 nCol = 32;
    };

    void setConfiguration(const ftConfig& config);

    void init() override;

    void shutdown() override;

    ftBroadphaseHandle addShape(const ftShape* shape, const ftTransform& transform, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform) override;

    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) override;

    void regionQuery(const ftAABB& region, ftChunkArray<const void*>* results) override;

    int getMemoryUsage() override;
private:

    struct ftElem {
        ftAABB aabb;
        int32 xStart, yStart, xEnd, yEnd;
        union {
            const void *userdata;
            int32 next;
        };
    };

    const static int32 NULL_NODE;

    ftConfig m_config;
    uint32 m_nBucket;

    ftChunkArray<uint32> *m_buckets;
    ftChunkArray<ftElem> m_elements;
    ftBitSet m_bucketMask; // use when inserting object to prevent adding the same object on the same grid

    int32 m_free;

    void insertToBucket(ftBroadphaseHandle handle);
    void removeFromBucket(ftBroadphaseHandle handle);
};


#endif //FALTON_FTTOROIDALGRID_H
