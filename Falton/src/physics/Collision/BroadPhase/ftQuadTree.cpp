//
// Created by Kevin Yu on 2016-05-23.
//

#include <falton/physics/collision/ftCollisionSystem.h>
#include "falton/physics/collision/broadphase/ftQuadTree.h"

#include <iostream>
using namespace std;

void ftQuadTree::setConfiguration(const ftConfig &config) {
    m_config = config;
}

void ftQuadTree::init() {
    m_pFreeNode = nullptr;
    m_pRoot = nullptr;
    m_iFreeObject = -1;

    m_objects.init(64);
    m_nodes.init(64);

    m_pRoot = allocateNode();
    m_pRoot->center = (m_config.worldAABB.max + m_config.worldAABB.min) / 2;
    m_pRoot->halfWidth = (m_config.worldAABB.max - m_config.worldAABB.min) / 2;

    m_nElem = 0;
}

void ftQuadTree::shutdown() {
    m_objects.cleanup();
    m_nodes.cleanup();
}

ftBroadphaseHandle ftQuadTree::addShape(const ftCollisionShape *const colShape, const void *const userData) {

    int32 iObj = allocateObject();
    ftObject* pObj = &m_objects[iObj];
    pObj->userdata = userData;
    pObj->aabb = colShape->shape->constructAABB(colShape->transform);
    insertObjectToTree(pObj, m_pRoot);

    ++m_nElem;

    return iObj;
}

void ftQuadTree::removeShape(ftBroadphaseHandle handle) {
    removeObjectFromTree(&m_objects[handle]);
    freeObject(handle);
    --m_nElem;
}

void ftQuadTree::moveShape(ftBroadphaseHandle handle, const ftCollisionShape &collisionShape) {

    m_objects[handle].aabb = collisionShape.shape->constructAABB(collisionShape.transform);

    ftObject* pObj = &m_objects[handle];
    ftNode* pNode = pObj->pNode;

    if (!isNodeEnclosingObject(pNode, pObj)) {
        removeObjectFromTree(pObj);
        insertObjectToTree(pObj, m_pRoot);
    } else if (getNodeQuarterArea(pNode) > m_objects[handle].aabb.getArea()){
        //node might previously straddle and now can go deeper in the tree
        removeObjectFromTree(pObj);
        insertObjectToTree(pObj, pNode);
    }
}

void ftQuadTree::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {

    TraversePoint traverseStack[30];
    int stackSize = 0;

    traverseStack[stackSize].pNode = m_pRoot;
    traverseStack[stackSize].childCount = 0;
    ++stackSize;

    findPairsInNode(traverseStack[0].pNode, pairs);

    //find pairs by depth first search
    while (stackSize > 0) {
        TraversePoint* traversePoint = &traverseStack[stackSize - 1];
        ftNode* pNode = traverseStack[stackSize-1].pNode;
        int32 childCount = traverseStack[stackSize-1].childCount;
        if (traversePoint->childCount < 4) {
            ftNode* pChild = pNode->pChild[childCount];
            if (pChild != nullptr) {
                for (int32 i = 0; i < stackSize; ++i) {
                    findPairsBetweenNodes(traverseStack[i].pNode, pChild, pairs);
                }
                findPairsInNode(pChild, pairs);

                traverseStack[stackSize].pNode = pNode->pChild[childCount];
                traverseStack[stackSize].childCount = 0;
                ++stackSize;
            }

            ++traversePoint->childCount;

        } else {
            --stackSize;
        }
    }
}

void ftQuadTree::findPairsBetweenNodes(ftNode* pNode1, ftNode* pNode2, ftChunkArray<ftBroadPhasePair>* pairs) {
    for (ftObject* pObj1 = pNode1->pObjects; pObj1 != nullptr;
            pObj1 = pObj1->pNext) {
        for (ftObject* pObj2 = pNode2->pObjects; pObj2 != nullptr;
                pObj2 = pObj2->pNext) {
            if (pObj1->aabb.overlap(pObj2->aabb)) {
                int32 index = pairs->add();
                (*pairs)[index].userdataA = pObj1->userdata;
                (*pairs)[index].userdataB = pObj2->userdata;
            }
        }
    }
}

void ftQuadTree::findPairsInNode(ftNode* pNode, ftChunkArray<ftBroadPhasePair>* pairs) {
    if (pNode->pObjects == nullptr) return;

    for (ftObject* pObj1 = pNode->pObjects; pObj1->pNext != nullptr;
            pObj1 = pObj1->pNext) {
        for (ftObject* pObj2 = pObj1->pNext; pObj2 != nullptr;
                pObj2 = pObj2->pNext) {
            if (pObj1->aabb.overlap(pObj2->aabb)) {
                int32 index = pairs->add();
                (*pairs)[index].userdataA = pObj1->userdata;
                (*pairs)[index].userdataB = pObj2->userdata;
            }
        }
    }
}

void ftQuadTree::insertObjectToTree(ftObject *pObj, ftNode *pRoot) {

    int32 locationCode = 0;
    ftVector2 childCenter = pRoot->center - pRoot->halfWidth / 2;
    bool stradle = false;

    if (pObj->aabb.min.x > pRoot->center.x) {
        locationCode |= 1;
        childCenter.x = pRoot->center.x + pRoot->halfWidth.x / 2;
    } else if (pObj->aabb.max.x > pRoot->center.x) {
        stradle = true;
    }

    if (pObj->aabb.min.y > pRoot->center.y) {
        locationCode |= 1 << 1;
        childCenter.y = pRoot->center.y + pRoot->halfWidth.y / 2;
    } else if (pObj->aabb.max.y > pRoot->center.y) {
        stradle = true;
    }

    if (stradle) {
        pObj->pNext = pRoot->pObjects;
        pObj->pNode = pRoot;
        pRoot->pObjects = pObj;
    } else {
        if (pRoot->pChild[locationCode] ==  nullptr) {
            ftNode* pNode = allocateNode();
            pRoot->pChild[locationCode] = pNode;
            pNode->center = childCenter;
            pNode->halfWidth = pRoot->halfWidth / 2;
        }
        insertObjectToTree(pObj, pRoot->pChild[locationCode]);
    }

}

void ftQuadTree::removeObjectFromTree(ftObject* pObject) {
    ftNode* pNode = pObject->pNode;
    ftObject** indirect = &(pNode->pObjects);

    while (*indirect != pObject) indirect = &((*indirect)->pNext);
    *indirect = (*indirect)->pNext;

}

ftQuadTree::ftNode* ftQuadTree::allocateNode() {
    ftNode* pNewNode;
    if (m_pFreeNode == nullptr) {
        int32 index = m_nodes.add();
        pNewNode = &m_nodes[index];
    } else {
        pNewNode = m_pFreeNode;
        m_pFreeNode = m_pFreeNode->pNext;
    }

    pNewNode->pChild[0] = nullptr;
    pNewNode->pChild[1] = nullptr;
    pNewNode->pChild[2] = nullptr;
    pNewNode->pChild[3] = nullptr;
    pNewNode->pObjects = nullptr;

    return pNewNode;

}

void ftQuadTree::freeNode(ftNode* pNode) {
    pNode->pNext = m_pFreeNode;
    m_pFreeNode = pNode;
}

int32 ftQuadTree::allocateObject() {
    int32 iNewObj;
    if (m_iFreeObject == NULL_OBJECT) {
        iNewObj = m_objects.add();
    } else {
        iNewObj = m_iFreeObject;
        m_iFreeObject = m_objects[m_iFreeObject].iNext;
    }

    return iNewObj;
}

void ftQuadTree::freeObject(int32 iObj) {
    m_objects[iObj].iNext = m_iFreeObject;
    m_iFreeObject = iObj;
}

bool ftQuadTree::isNodeEnclosingObject(ftNode *pNode, ftObject *pObj) {

    ftVector2 min = pNode->center - pNode->halfWidth;
    ftVector2 max = pNode->center + pNode->halfWidth;
    if (min.x < pObj->aabb.min.x && min.y < pObj->aabb.min.y
        && max.x > pObj->aabb.max.x && max.y > pObj->aabb.max.y) {
        return true;
    }
    return false;
}

int32 ftQuadTree::getNodeQuarterArea(ftNode* pNode) {
    return pNode->halfWidth.x * pNode->halfWidth.y;
}

