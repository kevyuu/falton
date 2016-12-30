//
// Created by Kevin Yu on 12/29/15.
//
#pragma once

#include <falton/shape/ftShape.h>

typedef uint32 ftBroadphaseHandle;

struct ftCollisionShape;

struct ftBroadPhasePair
{
    const void *userdataA;
    const void *userdataB;
};

class ftBroadphaseSystem
{
  public:
    ftBroadphaseSystem() {}
    virtual ~ftBroadphaseSystem() {}

    virtual void init() = 0;
    virtual void shutdown() = 0;
    virtual ftBroadphaseHandle addShape(const ftShape *colShape,
                                        const ftTransform &transform,
                                        const void *const userData) = 0;
    virtual void removeShape(ftBroadphaseHandle handle) = 0;
    virtual void moveShape(ftBroadphaseHandle handle,
                           const ftShape *shape,
                           const ftTransform &transform) = 0;

    virtual void findPairs(ftChunkArray<ftBroadPhasePair> *pairs) = 0;
    virtual void regionQuery(const ftAABB &region, ftChunkArray<const void *> *results) = 0;

    virtual int getMemoryUsage() = 0;

  protected:
};
