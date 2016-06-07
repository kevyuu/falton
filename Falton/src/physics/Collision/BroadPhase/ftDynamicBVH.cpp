//
// Created by Kevin Yu on 2016-05-12.
//

#include "falton/physics/collision/broadphase/ftDynamicBVH.h"
#include <falton/physics/collision/ftCollisionSystem.h>
#include <falton/container/ftBitSet.h>

#include <iostream>

using namespace std;

void ftDynamicBVH::setConfiguration(const ftConfig &config) {
    aabbExtension = config.aabbExtension;
}

void ftDynamicBVH::init() {
    m_root = NULL_NODE;
    m_free = NULL_NODE;

    m_nodes.init(64);
    m_moveBuffer.init(64);
}

void ftDynamicBVH::shutdown() {
    m_nodes.cleanup();
    m_moveBuffer.cleanup();
}

ftBroadphaseHandle ftDynamicBVH::addShape(const ftCollisionShape *const colShape, const void *const userData) {


    int index = allocateNode();

    m_nodes[index].aabb = colShape->shape->constructAABB(colShape->transform);
    m_nodes[index].aabb.extend(aabbExtension);
    m_nodes[index].userData = userData;

    insertLeaf(index);

    return index;
}

void ftDynamicBVH::moveShape(ftBroadphaseHandle handle, const ftCollisionShape &collisionShape) {

    ftAABB aabb = collisionShape.shape->constructAABB(collisionShape.transform);
    if (!m_nodes[handle].aabb.isContain(aabb)) {
        removeLeaf(handle);

        m_nodes[handle].aabb = aabb;
        m_nodes[handle].aabb.extend(aabbExtension);

        insertLeaf(handle);
    }
}

void ftDynamicBVH::removeShape(ftBroadphaseHandle handle) {
    removeLeaf(handle);
    freeNode(handle);
}

void ftDynamicBVH::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {
    for (uint32 i = 0; i < m_nodes.getSize(); ++i) {
        if (m_nodes[i].height > 0) computePairs(i, pairs);
    }

}

void ftDynamicBVH::computePairs(uint32 root, ftChunkArray<ftBroadPhasePair> *pairs) {
    if (isLeaf(root)) return;

    struct ftNodePair {
        uint32 idxA;
        uint32 idxB;
    };

    uint32 nNode = m_nodes.getSize();
    ftNodePair* stack = new ftNodePair[nNode * 3];
    uint32 stackSize = 0;

    stack[stackSize].idxA = m_nodes[root].leftChild;
    stack[stackSize].idxB = m_nodes[root].rightChild;
    ++stackSize;

    while (stackSize > 0) {
        uint32 idxA = stack[stackSize-1].idxA;
        uint32 idxB = stack[stackSize-1].idxB;
        --stackSize;

        ftAABB* aabbA = &m_nodes[idxA].aabb;
        ftAABB* aabbB = &m_nodes[idxB].aabb;

        if (!aabbA->overlap(*aabbB)) continue;
        if (isLeaf(idxA) && isLeaf(idxB)) {
            uint32 pairIdx = pairs->add();
            (*pairs)[pairIdx].userdataA = m_nodes[idxA].userData;
            (*pairs)[pairIdx].userdataB = m_nodes[idxB].userData;
        }
        else if (isLeaf(idxB) || (!isLeaf(idxA) && aabbA->getPerimeter() > aabbB->getPerimeter())) {
            stack[stackSize].idxA = m_nodes[idxA].leftChild;
            stack[stackSize].idxB = idxB;
            ++stackSize;

            stack[stackSize].idxA = m_nodes[idxA].rightChild;
            stack[stackSize].idxB = idxB;
            ++stackSize;
        } else {
            stack[stackSize].idxA = idxA;
            stack[stackSize].idxB = m_nodes[idxB].leftChild;
            ++stackSize;

            stack[stackSize].idxA = idxA;
            stack[stackSize].idxB = m_nodes[idxB].rightChild;
            ++stackSize;

        }
    }

    delete[] stack;

}

int32 ftDynamicBVH::allocateNode(){
    if (m_free == NULL_NODE) {
        int32 index = (int32) m_nodes.add();
        return index;
    } else {
        int32 index = m_free;
        m_free = m_nodes[index].next;
        return index;
    }

}

void ftDynamicBVH::freeNode(int32 idx) {
    m_nodes[idx].next = m_free;
    m_nodes[idx].height = -1;
    m_free = idx;
}

void ftDynamicBVH::insertLeaf(int32 newIndex) {
    m_nodes[newIndex].height = 0;
    m_nodes[newIndex].leftChild = NULL_NODE;
    m_nodes[newIndex].rightChild = NULL_NODE;
    m_nodes[newIndex].parent = NULL_NODE;

    if (m_root == NULL_NODE) {
        m_root = newIndex;
        return;
    }

    ftNode* node = &m_nodes[newIndex];

    int32 iterIndex = m_root;
    while(!isLeaf(iterIndex)) {

        real combinedPerimeter = ftAABB::combine(m_nodes[iterIndex].aabb, node->aabb).getPerimeter();
        real costParent = 2 * combinedPerimeter;
        real costInherit = 2 * combinedPerimeter - m_nodes[iterIndex].aabb.getPerimeter();

        int32 rightIndex = m_nodes[iterIndex].rightChild;
        int32 leftIndex = m_nodes[iterIndex].leftChild;

        real costRight = costInherit;
        ftNode* rightNode = &m_nodes[rightIndex];
        if (isLeaf(rightIndex)) {
            ftAABB aabb = ftAABB::combine(rightNode->aabb, node->aabb);
            costRight += aabb.getPerimeter();
        } else {
            ftAABB aabb = ftAABB::combine(rightNode->aabb, node->aabb);
            costRight += (aabb.getPerimeter() - rightNode->aabb.getPerimeter());
        }

        ftNode* leftNode = &m_nodes[leftIndex];
        real costLeft = costInherit;
        if (isLeaf(leftIndex)) {
            ftAABB aabb = ftAABB::combine(leftNode->aabb, node->aabb);
            costLeft += aabb.getPerimeter();
        } else {
            ftAABB aabb = ftAABB::combine(leftNode->aabb, node->aabb);
            costLeft += (aabb.getPerimeter() - leftNode->aabb.getPerimeter());
        }

        if (costParent < costRight && costParent <  costLeft) break;

        if (costRight < costLeft) {
            iterIndex = rightIndex;
        } else {
            iterIndex = leftIndex;
        }

    }

    int32 oldParent = m_nodes[iterIndex].parent;
    int32 newParent = allocateNode();

    m_nodes[newParent].parent = oldParent;
    m_nodes[newParent].leftChild = newIndex;
    m_nodes[newParent].rightChild = iterIndex;
    m_nodes[newParent].height = 1 + m_nodes[iterIndex].height;
    m_nodes[newParent].aabb = ftAABB::combine(node->aabb, m_nodes[iterIndex].aabb);

    m_nodes[iterIndex].parent = newParent;
    m_nodes[newIndex].parent = newParent;

    if (oldParent != NULL_NODE) {
        if (m_nodes[oldParent].leftChild == iterIndex) {
            m_nodes[oldParent].leftChild = newParent;
        } else {
            m_nodes[oldParent].rightChild = newParent;
        }
    } else {
        m_root = newParent;
    }

    balanceFromIndex(newParent);

}

void ftDynamicBVH::removeLeaf(int32 leafIdx) {
    int32 parentIdx = m_nodes[leafIdx].parent;

    if (parentIdx != NULL_NODE) {
        int32 grandParentIdx = m_nodes[parentIdx].parent;

        ftNode *parentNode = &m_nodes[parentIdx];
        int32 siblingIdx = parentNode->leftChild;
        if (parentNode->leftChild == leafIdx) {
            siblingIdx = parentNode->rightChild;
        }

        if (grandParentIdx != NULL_NODE) {
            ftNode *grandParentNode = &m_nodes[grandParentIdx];
            if (grandParentNode->leftChild == parentIdx) {
                grandParentNode->leftChild = siblingIdx;
            } else {
                grandParentNode->rightChild = siblingIdx;
            }
            m_nodes[siblingIdx].parent = grandParentIdx;
            recomputeHeightAndAABB(grandParentIdx);
            balanceFromIndex(grandParentIdx);
        } else {
            m_root = siblingIdx;
            m_nodes[siblingIdx].parent = NULL_NODE;
        }

        freeNode(parentIdx);


    } else {
        m_root = NULL_NODE;
    }

}

bool ftDynamicBVH::isLeaf(int32 index) {
    return (m_nodes[index].leftChild == NULL_NODE && m_nodes[index].rightChild == NULL_NODE);
}

int32 ftDynamicBVH::balanceFromIndex(int32 index) {
    while (index != NULL_NODE) {
        int32 leftIdx = m_nodes[index].leftChild;
        int32 rightIdx = m_nodes[index].rightChild;

        int32 parentIdx = m_nodes[index].parent;

        int32 diffHeight = m_nodes[rightIdx].height - m_nodes[leftIdx].height;
        int32 newIndex = index;
        if (diffHeight > 1) {
            newIndex = rotateLeft(index);
        } else if (diffHeight < -1) {
            newIndex = rotateRight(index);
        }

        if (parentIdx != NULL_NODE) {
            ftNode* parentNode = &m_nodes[parentIdx];
            if (m_nodes[parentIdx].leftChild == index) {
                parentNode->leftChild = newIndex;
            } else {
                parentNode->rightChild = newIndex;
            }
            recomputeHeightAndAABB(parentIdx);
        } else {
            m_root = newIndex;
        }

        index = parentIdx;

    }
    return 0;
}

int32 ftDynamicBVH::rotateLeft(int32 index) {
    ftNode* node = &m_nodes[index];

    int32 rightIdx = m_nodes[index].rightChild;
    ftNode* rightNode = &m_nodes[rightIdx];

    int32 grandLeftIdx = m_nodes[rightIdx].leftChild;
    int32 grandRightIdx = m_nodes[rightIdx].rightChild;
    int32 grandLeftHeight = m_nodes[grandLeftIdx].height;
    int32 grandRightHeight = m_nodes[grandRightIdx].height;

    rightNode->leftChild = index;
    rightNode->parent = node->parent;

    node->parent = rightIdx;

    if (grandRightHeight > grandLeftHeight) {
        rightNode->rightChild = grandRightIdx;
        node->rightChild = grandLeftIdx;
        m_nodes[grandLeftIdx].parent = index;
    } else {
        rightNode->rightChild = grandLeftIdx;
        node->rightChild = grandRightIdx;
        m_nodes[grandRightIdx].parent = index;
    }

    recomputeHeightAndAABB(index);
    recomputeHeightAndAABB(rightIdx);

    return rightIdx;
}

int32 ftDynamicBVH::rotateRight(int32 index) {
    ftNode* node = &m_nodes[index];

    int32 leftIdx = m_nodes[index].leftChild;
    ftNode* leftNode = &m_nodes[leftIdx];

    int32 grandLeftIdx = m_nodes[leftIdx].leftChild;
    int32 grandRightIdx = m_nodes[leftIdx].rightChild;
    int32 grandLeftHeight = m_nodes[grandLeftIdx].height;
    int32 grandRightHeight = m_nodes[grandRightIdx].height;
    ftNode* grandLeftNode = &m_nodes[grandLeftIdx];
    ftNode* grandRightNode = &m_nodes[grandRightIdx];

    leftNode->rightChild = index;
    leftNode->parent = node->parent;

    node->parent = leftIdx;

    if (grandRightHeight > grandLeftHeight) {
        leftNode->leftChild = grandRightIdx;
        node->leftChild = grandLeftIdx;
        grandLeftNode->parent = index;
    } else {
        leftNode->leftChild = grandLeftIdx;
        node->leftChild = grandRightIdx;
        grandRightNode->parent = index;
    }

    recomputeHeightAndAABB(index);
    recomputeHeightAndAABB(leftIdx);

    return leftIdx;
}


