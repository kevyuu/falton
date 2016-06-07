//
// Created by Kevin Yu on 2016-05-21.
//

#ifndef FALTON_FTTOROIDALGRID_H
#define FALTON_FTTOROIDALGRID_H


#include <falton/physics/collision/broadphase/ftBroadphaseSystem.h>

class ftToroidalGrid : public ftBroadphaseSystem {

public:

    struct ftConfig {
        real cellSize = 10;
        uint32 nRow = 8;
        uint32 nCol = 8;
    };

    void setConfiguration(const ftConfig& config);

    void init() override;

    void shutdown() override;

    ftBroadphaseHandle addShape(const ftCollisionShape *const colShape, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle, const ftCollisionShape &collisionShape) override;

    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) override;

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

    ftChunkArray<int> *m_buckets;
    ftChunkArray<ftElem> m_elements;

    int32 m_free;

    void insertToBucket(ftBroadphaseHandle handle);
    void removeFromBucket(ftBroadphaseHandle handle);
};


#endif //FALTON_FTTOROIDALGRID_H
