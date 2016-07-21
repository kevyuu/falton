//
// Created by Kevin Yu on 2016-05-12.
//

#include "falton/physics/collision/broadphase/ftDynamicBVH.h"
#include <falton/physics/collision/ftCollisionSystem.h>
#include <falton/container/ftBitSet.h>
#include <falton/container/ftStack.h>

void ftDynamicBVH::setConfiguration(const ftConfig &config) {
    aabbExtension = config.aabbExtension;
}

void ftDynamicBVH::init() {
    m_root = NULL_NODE;
    m_free = NULL_NODE;

    m_nodes.init(64);
}

void ftDynamicBVH::shutdown() {
    m_nodes.cleanup();
}

ftBroadphaseHandle ftDynamicBVH::addShape(const ftShape* shape, const ftTransform& transform, const void *const userData) {


    int index = allocateNode();

    m_nodes[index].aabb = shape->constructAABB(transform);
    m_nodes[index].aabb.extend(aabbExtension);
    m_nodes[index].userData = userData;

    insertLeaf(index);

    return index;
}

void ftDynamicBVH::moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform) {

    ftAABB aabb = shape->constructAABB(transform);
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

void ftDynamicBVH::regionQuery(const ftAABB& region, ftChunkArray<const void*>* results) {

    ftStack<uint32> stack;
    stack.init(m_nodes.getSize());

    stack.push(m_root);
    while (stack.getSize() > 0) {
        uint32 currentNode = stack.pop();
        if (currentNode == NULL_NODE) continue;
        if (region.overlap(m_nodes[currentNode].aabb)) {
            if (isLeaf(currentNode)) {
                uint32 index = results->push();
                (*results)[index] = m_nodes[currentNode].userData;
            } else {
                stack.push(m_nodes[currentNode].leftChild);
                stack.push(m_nodes[currentNode].rightChild);
            }
        }
    }

    stack.cleanup();

}

void ftDynamicBVH::computePairs(uint32 root, ftChunkArray<ftBroadPhasePair> *pairs) {
    if (isLeaf(root)) return;

    struct ftNodePair {
        uint32 idxA;
        uint32 idxB;
    };

    uint32 nNode = m_nodes.getSize();
    ftStack<ftNodePair> stack;
    stack.init(3 * nNode);

    stack.add();
    stack.top().idxA = m_nodes[root].leftChild;
    stack.top().idxB = m_nodes[root].rightChild;

    while (stack.getSize() > 0) {
        ftNodePair nodePair = stack.pop();

        ftAABB* aabbA = &m_nodes[nodePair.idxA].aabb;
        ftAABB* aabbB = &m_nodes[nodePair.idxB].aabb;

        if (!aabbA->overlap(*aabbB)) continue;
        if (isLeaf(nodePair.idxA) && isLeaf(nodePair.idxB)) {
            uint32 pairIdx = pairs->push();
            (*pairs)[pairIdx].userdataA = m_nodes[nodePair.idxA].userData;
            (*pairs)[pairIdx].userdataB = m_nodes[nodePair.idxB].userData;
        }
        else if (isLeaf(nodePair.idxB) || (!isLeaf(nodePair.idxA) && aabbA->getPerimeter() > aabbB->getPerimeter())) {
            stack.add();
            stack.top().idxA = m_nodes[nodePair.idxA].leftChild;
            stack.top().idxB = nodePair.idxB;

            stack.add();
            stack.top().idxA = m_nodes[nodePair.idxA].rightChild;
            stack.top().idxB = nodePair.idxB;

        } else {
            stack.add();
            stack.top().idxA = nodePair.idxA;
            stack.top().idxB = m_nodes[nodePair.idxB].leftChild;

            stack.add();
            stack.top().idxA = nodePair.idxA;
            stack.top().idxB = m_nodes[nodePair.idxB].rightChild;

        }
    }

    stack.cleanup();
}

uint32 ftDynamicBVH::allocateNode(){
    if (m_free == NULL_NODE) {
        uint32 index = (uint32) m_nodes.push();
        return index;
    } else {
        uint32 index = m_free;
        m_free = m_nodes[index].next;
        return index;
    }

}

void ftDynamicBVH::freeNode(uint32 idx) {
    m_nodes[idx].next = m_free;
    m_nodes[idx].height = -1;
    m_free = idx;
}

void ftDynamicBVH::insertLeaf(uint32 newIndex) {
    m_nodes[newIndex].height = 0;
    m_nodes[newIndex].leftChild = NULL_NODE;
    m_nodes[newIndex].rightChild = NULL_NODE;
    m_nodes[newIndex].parent = NULL_NODE;

    if (m_root == NULL_NODE) {
        m_root = newIndex;
        return;
    }

    ftNode* node = &m_nodes[newIndex];

    uint32 iterIndex = m_root;
    while(!isLeaf(iterIndex)) {

        real combinedPerimeter = ftAABB::combine(m_nodes[iterIndex].aabb, node->aabb).getPerimeter();
        real costParent = 2 * combinedPerimeter;
        real costInherit = 2 * combinedPerimeter - m_nodes[iterIndex].aabb.getPerimeter();

        uint32 rightIndex = m_nodes[iterIndex].rightChild;
        uint32 leftIndex = m_nodes[iterIndex].leftChild;

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

    uint32 oldParent = m_nodes[iterIndex].parent;
    uint32 newParent = allocateNode();

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

void ftDynamicBVH::removeLeaf(uint32 leafIdx) {
    uint32 parentIdx = m_nodes[leafIdx].parent;

    if (parentIdx != NULL_NODE) {
        uint32 grandParentIdx = m_nodes[parentIdx].parent;

        ftNode *parentNode = &m_nodes[parentIdx];
        uint32 siblingIdx = parentNode->leftChild;
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
            //balanceFromIndex(grandParentIdx);
        } else {
            m_root = siblingIdx;
            m_nodes[siblingIdx].parent = NULL_NODE;
        }

        freeNode(parentIdx);


    } else {
        m_root = NULL_NODE;
    }

}

bool ftDynamicBVH::isLeaf(uint32 index) {
    return (m_nodes[index].leftChild == NULL_NODE && m_nodes[index].rightChild == NULL_NODE);
}

uint32 ftDynamicBVH::balanceFromIndex(uint32 index) {
    while (index != NULL_NODE) {
        uint32 leftIdx = m_nodes[index].leftChild;
        uint32 rightIdx = m_nodes[index].rightChild;

        uint32 parentIdx = m_nodes[index].parent;

        int32 diffHeight = (int32) (m_nodes[rightIdx].height - m_nodes[leftIdx].height);
        uint32 newIndex = index;
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

uint32 ftDynamicBVH::rotateLeft(uint32 index) {
    ftNode* node = &m_nodes[index];

    uint32 rightIdx = m_nodes[index].rightChild;
    ftNode* rightNode = &m_nodes[rightIdx];

    uint32 grandLeftIdx = m_nodes[rightIdx].leftChild;
    uint32 grandRightIdx = m_nodes[rightIdx].rightChild;
    uint32 grandLeftHeight = m_nodes[grandLeftIdx].height;
    uint32 grandRightHeight = m_nodes[grandRightIdx].height;

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

uint32 ftDynamicBVH::rotateRight(uint32 index) {
    ftNode* node = &m_nodes[index];

    uint32 leftIdx = m_nodes[index].leftChild;
    ftNode* leftNode = &m_nodes[leftIdx];

    uint32 grandLeftIdx = m_nodes[leftIdx].leftChild;
    uint32 grandRightIdx = m_nodes[leftIdx].rightChild;
    uint32 grandLeftHeight = m_nodes[grandLeftIdx].height;
    uint32 grandRightHeight = m_nodes[grandRightIdx].height;
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


int ftDynamicBVH::getMemoryUsage() {
    return m_nodes.getSize() * sizeof(ftNode);
}


