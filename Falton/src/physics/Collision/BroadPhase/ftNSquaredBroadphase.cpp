//
// Created by Kevin Yu on 12/30/15.
//

#include "falton/physics/shape/ftShape.h"
#include "falton/physics/Collision/BroadPhase/ftNSquaredBroadphase.h"

void ftNSquaredBroadphase::init(ftTransformShape *collisionShapes) {
    this->collisionShapes = collisionShapes;
}

void ftNSquaredBroadphase::shutdown() {
    //do nothing
}

void ftNSquaredBroadphase::newShape(ColHandle handle) {

    ftTransformShape transformShape = collisionShapes[handle];

    aabbBuffers[handle] = transformShape.shape->constructAABB(transformShape.transform);
    dirtyHandles.push(handle);
    if (lastHandle < handle) lastHandle = handle;

}

void ftNSquaredBroadphase::removeShape(ColHandle handle) {
    aabbBuffers[handle].min = aabbBuffers[handle].max; // convention for empty aabb if aabb.min = aabb.max;
}

void ftNSquaredBroadphase::moveShape(ColHandle handle) {
    newShape(handle);
    dirtyHandles.push(handle);
}

void ftNSquaredBroadphase::findPairs(ftChunkArray<ftBroadPhasePair> &pairs) {

    ftBroadPhasePair pair;
    for (uint32 i=0;i<=lastHandle;i++) {
        for (uint32 j = i+1; j <= lastHandle; j++) {
            if (!(aabbBuffers[i].max == aabbBuffers[i].min) &&
                aabbBuffers[i].overlap(aabbBuffers[j])) {
                pair.handleA = i;
                pair.handleB = j;
                pairs.push(pair);
            }
        }
    }
}

