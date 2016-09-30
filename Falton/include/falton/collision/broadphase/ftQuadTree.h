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
        uint8 maxLevel = 15; //level only range from 0 to 15
    };

    void setConfiguration(const ftConfig& config);

    void init() override;

    void shutdown() override;

    ftBroadphaseHandle addShape(const ftShape *const shape, const ftTransform& transform, const void *const userData) override;

    void removeShape(ftBroadphaseHandle handle) override;

    void moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform) override;

    void findPairs(ftChunkArray<ftBroadPhasePair>* pairs) override;

    void regionQuery(const ftAABB& region, ftChunkArray<const void*>* results) override;

    int getMemoryUsage() override;

private:
    struct ftNode;

    struct ftObject {
        const void* userdata;
        ftAABB aabb;
        union {
            ftObject* pNext;
            int32 iNext; // use when object is free. Index to next free object;
        };
        ftNode* pNode;
    };

    struct ftNode {
        ftVector2 center;
        ftVector2 halfWidth;
        ftNode* pChild[4];
        ftNode* pParent;
        union {
            ftObject* pObjects;
            ftNode* pNext; // use when node is free. Pointer to next free node.
        };
        uint8 levelChild; // first four bits for level and the next four bits for number of child

        inline uint8 getLevel() {
            return (levelChild >> 4);
        }

        inline void setLevel(uint8 level) {
            levelChild &= 0x0F;
            levelChild |= (level << 4);
        }

        inline uint8 getNChild() {
            return (levelChild & 0x0F);
        }
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
    void removeNodeIfEmpty(ftNode* pNode);
    bool isNodeEnclosingObject(ftNode *pNode, ftObject *pObj);
    real getNodeQuarterArea(ftNode* pNode);

};


#endif //FALTON_FTQUADTREE_H
