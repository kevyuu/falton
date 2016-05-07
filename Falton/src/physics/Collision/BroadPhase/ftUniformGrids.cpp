//
// Created by Kevin Yu on 5/4/16.
//

#include <falton/physics/collision/ftCollisionSystem.h>
#include "falton/physics/collision/broadphase/ftUniformGrids.h"

void ftUniformGrids::setConfig(ftConfig config){
    this->config = config;
}

void ftUniformGrids::init() {
    nRow = (config.worldWidth / config.cellWidth) + 1;
    nCol = (config.worldHeight / config.cellHeight) + 1;
    grids = new ftElem*[nRow * nCol];
    proxies = new ftChunkArray<ftProxy>(64);
}

void ftUniformGrids::shutdown() {
    delete[] grids;
    delete proxies;
}

ftBroadphaseHandle ftUniformGrids::addShape(const ftCollisionShape *const colShape, const void *const userData) {
    uint32 handle;
    if (freeHandles.getSize() > 0) {
        handle = freeHandles.pop();
    } else {
        handle = proxies->add();
    }

    (*proxies)[handle].collisionShape = colShape;
    (*proxies)[handle].userdata = userData;

    return handle;

}

void ftUniformGrids::removeShape(ftBroadphaseHandle handle) {
    freeHandles.push(handle);

    (*proxies)[handle].collisionShape = nullptr;
}

void ftUniformGrids::moveShape(ftBroadphaseHandle handle) {
    // do nothing
}

void ftUniformGrids::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {

    for (uint32 i = 0; i < proxies->getSize(); ++i) {
        ftShape* shape = (*proxies)[i].collisionShape->shape;
        ftTransform transform = (*proxies)[i].collisionShape->transform;
        ftAABB aabb = shape->constructAABB(transform);

        uint32 minRow = aabb.min.y / config.cellHeight;
        uint32 minCol = aabb.min.x / config.cellWidth;

        uint32 maxRow = (aabb.max.y / config.cellHeight) + 1;
        uint32 maxCol = (aabb.max.x / config.cellWidth) + 1;

        for (uint32 j = minRow; j < maxRow; ++j) {
            for (uint32 k = minCol; k < maxCol; ++k) {
                ftElem *elem = new ftElem;
                elem->proxy = &(*proxies)[i];

                uint32 index = j * nCol + k;

                elem->next = grids[index];
                grids[index] = elem;
            }
        }
    }

}