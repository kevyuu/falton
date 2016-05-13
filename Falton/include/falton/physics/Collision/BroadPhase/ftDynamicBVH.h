//
// Created by Kevin Yu on 2016-05-12.
//

#ifndef FALTON_FTDYNAMICBVH_H
#define FALTON_FTDYNAMICBVH_H

#include <falton/setting/general.h>
#include <falton/physics/shape/ftAABB.h>
#include <falton/physics/collision/broadphase/ftBroadphaseSystem.h>

class ftDynamicBVH : public ftBroadphaseSystem {

public:
    ftDynamicBVH();

    ~ftDynamicBVH() override;

    void init() override;

    void shutdown() override;

    ftBroadphaseHandle addShape(const ftCollisionShape *const colShape, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle, const ftCollisionShape &collisionShape) override;

    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) override;

private:
    struct ftNode {

        ftAABB aabb;

        union {
            int32 parent;
            int32 next;
        };

        int32 leftChild;
        int32 rightChild;
        int32 height;

        const void *userData;
    };

    static constexpr int32 NULL_NODE = -1;

    ftChunkArray<ftNode> m_nodes;

    int32 m_root;
    int32 m_free;

    int32 allocateNode();
    void freeNode(int32 idx);

    void insertLeaf(int32 newIndex);
    void removeLeaf(int32 leafIdx);

    bool isLeaf(int32 index);

    int32 balanceFromIndex(int32 index);
    int32 rotateLeft(int32 index);
    int32 rotateRight(int32 index);

    void recomputeHeightAndAABB(int32 index);
};


inline void ftDynamicBVH::recomputeHeightAndAABB(int32 index) {
    int32 leftIdx = m_nodes[index].leftChild;
    int32 rightIdx = m_nodes[index].rightChild;

    int32 leftHeight = m_nodes[leftIdx].height;
    int32 rightHeight = m_nodes[rightIdx].height;
    m_nodes[index].height = ftMax(leftHeight, rightHeight) + 1;

    ftAABB* leftAABB = &(m_nodes[leftIdx].aabb);
    ftAABB* rightAABB = &(m_nodes[rightIdx].aabb);
    m_nodes[index].aabb = ftAABB::combine(*leftAABB, *rightAABB);
}

#endif //FALTON_FTDYNAMICBVH_H
