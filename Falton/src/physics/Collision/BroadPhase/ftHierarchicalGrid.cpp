//
// Created by Kevin Yu on 5/5/16.
//

#include <cstring>

#include <falton/physics/collision/ftCollisionSystem.h>
#include <falton/physics/collision/broadphase/ftHierarchicalGrid.h>

#include <iostream>
using namespace std;

ftHierarchicalGrid::ftHierarchicalGrid() {

    nLevel = 32;
    baseSize = 5;
    sizeMul = 2;
    bucketCapacity = 1024;

}

void ftHierarchicalGrid::setConfig(ftConfig config) {

    nLevel = config.nLevel;
    baseSize = config.baseSize;
    sizeMul = config.sizeMul;

}

void ftHierarchicalGrid::init() {

    elemList = new ftChunkArray<ftElem>(64);

    elemBucket = new ftElem*[bucketCapacity];
    for (uint32 i = 0; i < bucketCapacity; ++i) {
        elemBucket[i] = nullptr;
    }

    nObjectAtLevel = new uint32[nLevel];
    for (uint32 i = 0; i < nLevel; ++i) {
        nObjectAtLevel[i] = 0;
    }

    cellSizeTable = new real[nLevel];
    cellSizeTable[0] = baseSize;
    for (uint32 i = 1; i < nLevel; ++i) {
        cellSizeTable[i] = cellSizeTable[i-1] * sizeMul;
    }

}

void ftHierarchicalGrid::shutdown() {

    delete elemList;
    delete[] nObjectAtLevel;
    delete[] elemBucket;
    delete[] cellSizeTable;

}

ftBroadphaseHandle  ftHierarchicalGrid::addShape(const ftCollisionShape *const colShape, const void *const userData) {

    ftBroadphaseHandle handle;
    if (freeHandles.getSize() > 0) {
        handle = freeHandles.pop();
    } else {
        handle = elemList->add();
    }

    ftElem* elem = &(*elemList)[handle];
    elem->collisionShape = colShape;
    elem->userdata = userData;

    elem->aabb = colShape->shape->constructAABB(colShape->transform);

    real width = elem->aabb.max.x - elem->aabb.min.x;
    real height = elem->aabb.max.y - elem->aabb.max.y;

    uint32 level = 0;
    while (cellSizeTable[level] < width || cellSizeTable[level]< height) {
        ++level;
    }
    elem->level = level;
    real size = cellSizeTable[level];

    nObjectAtLevel[level]++;

    int32 gridX = floorf(elem->aabb.min.x / size);
    int32 gridY = floorf(elem->aabb.min.y / size);

    uint32 bucketIndex = computeHashIndex(gridX, gridY, level);

    insertElemToBucket(elem, bucketIndex);

    return handle;
}

void ftHierarchicalGrid::moveShape(ftBroadphaseHandle handle) {

    ftElem* elem = &(*elemList)[handle];
    const ftCollisionShape* colShape = (*elemList)[handle].collisionShape;

    elem->aabb = colShape->shape->constructAABB(colShape->transform);

    uint32 level = elem->level;
    real size = cellSizeTable[level];

    int32 gridX = floorf(elem->aabb.min.x / size);
    int32 gridY = floorf(elem->aabb.min.y / size);

    uint32 bucketIndex = computeHashIndex(gridX, gridY, level);

    if (bucketIndex != elem->bucketIndex) {
        unlink(elem);
        insertElemToBucket(elem,bucketIndex);
    }

}

void ftHierarchicalGrid::removeShape(ftBroadphaseHandle handle) {

    ftElem* elem = &(*elemList)[handle];

    unlink(elem);

    freeHandles.push(handle);

    elem->collisionShape = nullptr;

    --nObjectAtLevel[elem->level];

}

void ftHierarchicalGrid::unlink(ftElem *elem) {
    if (elem->prev != nullptr) {
        elem->prev->next = elem->next;
    } else {
        elemBucket[elem->bucketIndex] = elem->next;
    }

    if (elem->next != nullptr) {
        elem->next->prev = elem->prev;
    }
}

void ftHierarchicalGrid::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {

    ftBitSet bitMask(bucketCapacity);

    for (uint32 i = 0; i < elemList->getSize(); ++i) {
        bitMask.clear();

        ftElem* elem = &(*elemList)[i];

        if (elem->collisionShape == nullptr) continue;

        uint32 level = elem->level;
        real size = cellSizeTable[level];

        // check current level;
        {
            // check current cell;
            ftBroadPhasePair pair;
            for(ftElem* other = elem->next; other != nullptr; other = other->next) {
                if (elem->aabb.overlap(other->aabb)) {
                    uint32 index = pairs->add();
                    (*pairs)[index].userdataA = elem->userdata;
                    (*pairs)[index].userdataB = elem->userdata;
                }
            }
            bitMask.on(elem->bucketIndex);

            int32 gridX = floorf(elem->aabb.min.x / size);
            int32 gridY = floorf(elem->aabb.min.y / size);

            int16 xDirection[4] = {1 , 0 , 1 , -1};
            int16 yDirection[4] = {0 , 1 , 1 ,  1};

            // only check 4 neighbor because other elem in same level will check other direction
            for (uint16 j = 0; j < 4; ++j) {
                uint32 index = computeHashIndex(gridX + xDirection[j], gridY + yDirection[j], level);
                if (!bitMask.test(index)) {
                    addCollidingPairInBucket(elem, index, pairs);
                    bitMask.on(index);
                }
            }

        }

        ++level;

        int16 xDirection[8] = {1 ,  1 ,  0 , -1 , -1 , -1 , 0 , 1};
        int16 yDirection[8] = {0 , -1 , -1 , -1 ,  0 ,  1 , 1 , 1};

        while (level < nLevel) {
            size = cellSizeTable[level];

            if (nObjectAtLevel[level] == 0) {
                ++level;
                continue;
            }

            int32 gridX = floorf(elem->aabb.min.x / size);
            int32 gridY = floorf(elem->aabb.min.y / size);

            //check center cell;
            uint32 centerIndex = computeHashIndex(gridX, gridY, level);
            if (!bitMask.test(centerIndex)) {
                addCollidingPairInBucket(elem, centerIndex, pairs);
                bitMask.on(centerIndex);
            }

            //check neighbor cell;
            for (uint16 j = 0; j < 8; ++j) {
                uint32 index = computeHashIndex(gridX + xDirection[j] , gridY + yDirection[j], level);
                if (!bitMask.test(index)) {
                    addCollidingPairInBucket(elem, index, pairs);
                    bitMask.on(index);
                }
            }

            ++level;

        }
    }

}

void ftHierarchicalGrid::addCollidingPairInBucket(ftElem *elem, uint32 bucketIndex,
                                                  ftChunkArray<ftBroadPhasePair> *pairs) {
    for (ftElem* other = elemBucket[bucketIndex]; other != nullptr; other = other->next) {
        if (elem->aabb.overlap(other->aabb)) {
            uint32 index = pairs->add();
            (*pairs)[index].userdataA = elem->userdata;
            (*pairs)[index].userdataB = other->userdata;
        }
    }
}

uint32 ftHierarchicalGrid::computeHashIndex(int32 x, int32 y, uint32 level) {

    const int32 h1 = 0x8da6b343; // Large multiplicative constants;
    const int32 h2 = 0xd8163841; // here arbitrarily chosen primes
    const int32 h3 = 0xcb1ab31f;
    int32 n = h1 * x + h2 * y + h3 * level;
    n = n % bucketCapacity;
    return n;

}

void ftHierarchicalGrid::insertElemToBucket(ftElem *elem, uint32 bucketIndex) {
    elem->bucketIndex = bucketIndex;
    elem->prev = nullptr;
    elem->next = elemBucket[bucketIndex];
    if (elemBucket[bucketIndex] != nullptr) elemBucket[bucketIndex]->prev = elem;
    elemBucket[bucketIndex] =  elem;
}
