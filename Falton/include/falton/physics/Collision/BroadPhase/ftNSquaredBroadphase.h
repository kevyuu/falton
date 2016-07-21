//
// Created by Kevin Yu on 12/30/15.
//

#ifndef FALTON_FTNSQUAREDBROADPHASE_H
#define FALTON_FTNSQUAREDBROADPHASE_H

#include<queue>

#include <falton/physics/collision/broadphase/ftBroadphaseSystem.h>
#include <falton/container/ftIntQueue.h>
#include <falton/container/ftChunkArray.h>
#include <falton/physics/shape/ftAABB.h>

struct ftCollisionShape;

class ftNSquaredBroadphase : public ftBroadphaseSystem{
public:

    virtual int getMemoryUsage() override;

    ftNSquaredBroadphase() {};
    ~ftNSquaredBroadphase() {};

    void init();
    void shutdown();

    ftBroadphaseHandle addShape(const ftShape* const colShape, const ftTransform& transform, const void* const userData);
    void removeShape(ftBroadphaseHandle handle);
    void moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform);
    void findPairs(ftChunkArray<ftBroadPhasePair> *pairs);
    void regionQuery(const ftAABB& region, ftChunkArray<const void*>* results) override;

private:

    struct ftElem {
        ftAABB aabb;
        bool isAllocated;
        const void* userdata;
    };

    uint32 m_nShape;
    ftChunkArray<ftElem> m_elements;
    ftIntQueue m_freeHandleList;

};


#endif //FALTON_FTNSQUAREDBROADPHASE_H
