//
// Created by Kevin Yu on 2016-05-23.
//

#include <falton/collision/ftCollisionSystem.h>
#include <falton/collision/broadphase/ftQuadTree.h>
#include <falton/container/ftStack.h>

void ftQuadTree::setConfiguration(const ftConfig &config) {
    ftAssert(config.maxLevel <= 15, "Quad Tree level only range from 0 to 15. Current configuration max level : "<<m_config.maxLevel);
    m_config = config;
}

void ftQuadTree::init() {
    m_pFreeNode = nullptr;
    m_pRoot = nullptr;
    m_iFreeObject = -1;

    m_objects.init(128);
    m_nodes.init(1024);

    m_pRoot = allocateNode();
    m_pRoot->center = (m_config.worldAABB.max + m_config.worldAABB.min) / 2;
    m_pRoot->halfWidth = (m_config.worldAABB.max - m_config.worldAABB.min) / 2;

    m_nElem = 0;
}

void ftQuadTree::shutdown() {
    m_objects.cleanup();
    m_nodes.cleanup();
}

ftBroadphaseHandle ftQuadTree::addShape(const ftShape* shape, const ftTransform& transform, const void *const userData) {

    int32 iObj = allocateObject();
    ftObject* pObj = &m_objects[iObj];
    pObj->userdata = userData;
    pObj->aabb = shape->constructAABB(transform);
    ftAssert(isNodeEnclosingObject(m_pRoot, pObj), "");
    insertObjectToTree(pObj, m_pRoot);

    ++m_nElem;

    return iObj;
}

void ftQuadTree::removeShape(ftBroadphaseHandle handle) {
    ftNode* pNode = m_objects[handle].pNode;
    removeObjectFromTree(&m_objects[handle]);
    freeObject(handle);
    --m_nElem;
    removeNodeIfEmpty(pNode);
}

void ftQuadTree::moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform) {

    m_objects[handle].aabb = shape->constructAABB(transform);
    ftAssert(isNodeEnclosingObject(m_pRoot, &m_objects[handle]),"");

    ftObject* pObj = &m_objects[handle];
    ftNode* pNode = pObj->pNode;

    if (!isNodeEnclosingObject(pNode, pObj)) {
        removeObjectFromTree(pObj);
        insertObjectToTree(pObj, m_pRoot);
        ftNode* nodeToRemove = pNode;
        while (nodeToRemove != m_pRoot) {
            ftNode* parent = nodeToRemove->pParent;
            removeNodeIfEmpty(nodeToRemove);
            nodeToRemove = parent;
        }
    } else if (getNodeQuarterArea(pNode) > m_objects[handle].aabb.getArea()){
        //node might previously straddle and now can go deeper in the tree
        removeObjectFromTree(pObj);
        insertObjectToTree(pObj, pNode);
    }
}

void ftQuadTree::regionQuery(const ftAABB &region, ftChunkArray<const void *>* results) {

    const auto isAABBOverlapWithNode = [] (const ftAABB& aabb, const ftNode& node) -> bool {
        ftVector2 min = node.center - node.halfWidth;
        ftVector2 max = node.center + node.halfWidth;
        if (max.x < aabb.min.x || aabb.max.x < min.x) return false;
        if (max.y< aabb.min.y || aabb.max.y < min.y) return false;
        return true;
    };

    ftStack<ftNode*> stack;
    stack.init(2 * m_nodes.getSize());

    stack.push(m_pRoot);

    while (stack.getSize() > 0) {
        ftNode* pNode = stack.pop();
        if (pNode != nullptr && isAABBOverlapWithNode(region, *pNode)) {
            for (ftObject* pObject = pNode->pObjects; pObject != nullptr; pObject = pObject->pNext) {
                if (pObject->aabb.overlap(region)) {
                    results->push(pObject->userdata);
                }
            }
            for (uint32 i = 0 ; i < 4; ++i) {
                stack.push(pNode->pChild[i]);
            }
        }
    }

    stack.cleanup();

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
                int32 index = pairs->push();
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
                int32 index = pairs->push();
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

    if (stradle || pRoot->getLevel() >= m_config.maxLevel) {
        pObj->pNext = pRoot->pObjects;
        pObj->pNode = pRoot;
        pRoot->pObjects = pObj;
    } else {
        if (pRoot->pChild[locationCode] ==  nullptr) {
            ftNode* pNode = allocateNode();
            pRoot->pChild[locationCode] = pNode;
            ++pRoot->levelChild;
            pNode->pParent = pRoot;
            pNode->center = childCenter;
            pNode->halfWidth = pRoot->halfWidth / 2;
            pNode->setLevel(pRoot->getLevel() + 1);
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

void ftQuadTree::removeNodeIfEmpty(ftNode *pNode) {
    if (pNode->pObjects == nullptr && pNode->getNChild() == 0) {
        ftNode* pParent = pNode->pParent;
        --pParent->levelChild;
        for (uint8 i = 0 ; i < 4; ++i) {
            if (pParent->pChild[i] == pNode) pParent->pChild[i] = nullptr;
        }
        freeNode(pNode);
    }
}

ftQuadTree::ftNode* ftQuadTree::allocateNode() {
    ftNode* pNewNode;
    if (m_pFreeNode == nullptr) {
        int32 index = m_nodes.push();
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
    pNewNode->pParent = nullptr;
    pNewNode->levelChild = 0;

    return pNewNode;

}

void ftQuadTree::freeNode(ftNode* pNode) {
    pNode->pNext = m_pFreeNode;
    m_pFreeNode = pNode;
}

int32 ftQuadTree::allocateObject() {
    int32 iNewObj;
    if (m_iFreeObject == NULL_OBJECT) {
        iNewObj = m_objects.push();
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

real ftQuadTree::getNodeQuarterArea(ftNode* pNode) {
    return pNode->halfWidth.x * pNode->halfWidth.y;
}


int ftQuadTree::getMemoryUsage() {
    return (m_objects.getSize() * sizeof(ftObject)) + (m_nodes.getSize() * sizeof(ftNode));
}

