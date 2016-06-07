//
// Created by Kevin Yu on 2016-05-23.
//

#ifndef FALTON_FTQUADTREE_H
#define FALTON_FTQUADTREE_H


#include <falton/physics/collision/broadphase/ftBroadphaseSystem.h>

class ftQuadTree : public ftBroadphaseSystem {

public:

    struct ftConfig {
        ftAABB worldAABB;
    };

    void setConfiguration(const ftConfig& config);

    void init() override;

    void shutdown() override;

    ftBroadphaseHandle addShape(const ftCollisionShape *const colShape, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle, const ftCollisionShape &collisionShape) override;

    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) override;

private:
    struct ftNode;

    struct ftObject {
        const void* userdata;
        ftAABB aabb;
        union {
            int32 iNext;
            ftObject* pNext;
        };
        ftNode* pNode;
    };

    struct ftNode {
        ftVector2 center;
        ftVector2 halfWidth;
        ftNode* pChild[4];
        union {
            ftObject* pObjects;
            ftNode* pNext;
        };
    };

    struct TraversePoint{
        ftNode* pNode;
        int32 childCount;
    };

    static const int32 NULL_OBJECT = -1;
    ftConfig m_config;

    ftChunkArray<ftObject> m_objects;
    ftChunkArray<ftNode> m_nodes;

    int32 m_iFreeObject;
    ftNode* m_pRoot;
    ftNode* m_pFreeNode;

    int32 m_nElem;

    ftNode* allocateNode();
    void freeNode(ftNode* pNode);
    int32 allocateObject();
    void freeObject(int32 iObj);

    void findPairsBetweenNodes(ftNode* pNode1, ftNode* pNode2, ftChunkArray<ftBroadPhasePair>* pairs);
    void findPairsInNode(ftNode* pNode, ftChunkArray<ftBroadPhasePair>* pairs);

    void insertObjectToTree(ftObject *pObj, ftNode *pRootNode);
    void removeObjectFromTree(ftObject* pObj);
    bool isNodeEnclosingObject(ftNode *pNode, ftObject *pObj);
    int32 getNodeQuarterArea(ftNode* pNode);

};


#endif //FALTON_FTQUADTREE_H
