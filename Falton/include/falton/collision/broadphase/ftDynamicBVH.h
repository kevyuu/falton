//
// Created by Kevin Yu on 2016-05-12.
//

#ifndef FALTON_FTDYNAMICBVH_H
#define FALTON_FTDYNAMICBVH_H

#include <falton/setting.h>
#include <falton/shape/ftAABB.h>
#include <falton/collision/broadphase/ftBroadphaseSystem.h>

class ftDynamicBVH : public ftBroadphaseSystem {

public:

    struct ftConfig {
        real aabbExtension = 0.05;
    };

    ftDynamicBVH() {};

    ~ftDynamicBVH() override {};

    void init() override;
    void shutdown() override;

    void setConfiguration(const ftConfig& config);

    ftBroadphaseHandle addShape(const ftShape* shape, const ftTransform& transform, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform) override;

    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) override;

    void regionQuery(const ftAABB& region, ftChunkArray<const void*>* results) override;

    int getMemoryUsage() override;

private:
    struct ftNode {

        ftAABB aabb;

        union {
            uint32 parent;
            uint32 next;
        };

        uint32 leftChild;
        uint32 rightChild;
        uint32 height;

        const void *userData;
    };

    static constexpr uint32 NULL_NODE = nulluint;

    ftChunkArray<ftNode> m_nodes;

    uint32 m_root;
    uint32 m_free;

    uint32 allocateNode();
    void freeNode(uint32 idx);

    void insertLeaf(uint32 newIndex);
    void removeLeaf(uint32 leafIdx);

    bool isLeaf(uint32 index);

    uint32 balanceFromIndex(uint32 index);
    uint32 rotateLeft(uint32 index);
    uint32 rotateRight(uint32 index);

    void recomputeHeightAndAABB(uint32 index);

    void computePairs(uint32 root, ftChunkArray<ftBroadPhasePair> *pairs);

    //configuration
    real aabbExtension;
};


inline void ftDynamicBVH::recomputeHeightAndAABB(uint32 index) {
    uint32 leftIdx = m_nodes[index].leftChild;
    uint32 rightIdx = m_nodes[index].rightChild;

    uint32 leftHeight = m_nodes[leftIdx].height;
    uint32 rightHeight = m_nodes[rightIdx].height;

    m_nodes[index].height = ftMax(leftHeight, rightHeight) + 1;

    ftAABB* leftAABB = &(m_nodes[leftIdx].aabb);
    ftAABB* rightAABB = &(m_nodes[rightIdx].aabb);
    m_nodes[index].aabb = ftAABB::combine(*leftAABB, *rightAABB);
}

#endif //FALTON_FTDYNAMICBVH_H
