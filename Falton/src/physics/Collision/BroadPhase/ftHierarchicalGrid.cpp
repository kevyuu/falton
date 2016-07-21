//
// Created by Kevin Yu on 5/5/16.
//

#include <cstring>

#include <falton/physics/collision/ftCollisionSystem.h>
#include <falton/physics/collision/broadphase/ftHierarchicalGrid.h>

void ftHierarchicalGrid::setConfiguration(ftConfig config) {

    m_nLevel = config.nLevel;
    m_baseSize = config.baseSize;
    m_sizeMul = config.sizeMul;
    m_bucketCapacity = config.nBucket;

}

void ftHierarchicalGrid::init() {

    m_elemList.init(64);

    m_elemBucket = new ftElem*[m_bucketCapacity];
    for (uint32 i = 0; i < m_bucketCapacity; ++i) {
        m_elemBucket[i] = nullptr;
    }

    m_nObject = new uint32[m_nLevel];
    for (uint32 i = 0; i < m_nLevel; ++i) {
        m_nObject[i] = 0;
    }

    m_cellSizeTable = new real[m_nLevel];
    m_cellSizeTable[0] = m_baseSize;
    for (uint32 i = 1; i < m_nLevel; ++i) {
        m_cellSizeTable[i] = m_cellSizeTable[i-1] * m_sizeMul;
    }

    m_freeElem = NULL_ELEM;
}

void ftHierarchicalGrid::shutdown() {

    m_elemList.cleanup();

    delete[] m_elemBucket;
    delete[] m_nObject;
    delete[] m_cellSizeTable;

}

ftBroadphaseHandle  ftHierarchicalGrid::addShape(const ftShape* shape, const ftTransform& transform, const void *const userData) {

    ftBroadphaseHandle handle;
    if (m_freeElem == NULL_ELEM) {
        handle = m_elemList.push();
    } else {
        handle = m_freeElem;
        m_freeElem = m_elemList[handle].nextFree;
    }
    ftElem* elem = &m_elemList[handle];
    elem->userdata = userData;

    elem->aabb = shape->constructAABB(transform);

    real width = elem->aabb.max.x - elem->aabb.min.x;
    real height = elem->aabb.max.y - elem->aabb.min.y;

    uint32 level = 0;
    while (m_cellSizeTable[level] < width || m_cellSizeTable[level]< height) {
        ++level;
    }
    elem->level = level;
    real size = m_cellSizeTable[level];

    m_nObject[level]++;

    int32 gridX = ftFloor(elem->aabb.min.x / size);
    int32 gridY = ftFloor(elem->aabb.min.y / size);

    uint32 bucketIndex = computeHashIndex(gridX, gridY, level);

    insertElemToBucket(elem, bucketIndex);

    return handle;
}

void ftHierarchicalGrid::moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform) {

    ftElem* elem = &m_elemList[handle];

    elem->aabb = shape->constructAABB(transform);

    uint32 level = elem->level;
    real size = m_cellSizeTable[level];

    int32 gridX = ftFloor(elem->aabb.min.x / size);
    int32 gridY = ftFloor(elem->aabb.min.y / size);

    uint32 bucketIndex = computeHashIndex(gridX, gridY, level);

    if (bucketIndex != elem->bucketIndex) {
        unlink(elem);
        insertElemToBucket(elem,bucketIndex);
    }

}

void ftHierarchicalGrid::removeShape(ftBroadphaseHandle handle) {

    ftElem* elem = &m_elemList[handle];

    unlink(elem);

    m_elemList[handle].nextFree = m_freeElem;
    m_freeElem = handle;

    elem->userdata = nullptr;

    --m_nObject[elem->level];

}

void ftHierarchicalGrid::unlink(ftElem *elem) {
    if (elem->prev != nullptr) {
        elem->prev->next = elem->next;
    } else {
        m_elemBucket[elem->bucketIndex] = elem->next;
    }

    if (elem->next != nullptr) {
        elem->next->prev = elem->prev;
    }
}

void ftHierarchicalGrid::regionQuery(const ftAABB &region, ftChunkArray<const void*>* results) {
    ftAssert(region.getArea() < m_cellSizeTable[m_nLevel-1],"");

    ftBitSet gridFlag;
    gridFlag.init(m_bucketCapacity);

    for (uint32 level = 0; level < m_nLevel; ++level) {
        if (m_nObject[level] == 0) continue;
        int32 startGridX = ftFloor(region.min.x / m_cellSizeTable[level]) - 1;
        int32 startGridY = ftFloor(region.min.y / m_cellSizeTable[level]) - 1;
        int32 endGridX = ftFloor(region.max.x / m_cellSizeTable[level]);
        int32 endGridY = ftFloor(region.max.y / m_cellSizeTable[level]);

        for (int32 gridX = startGridX; gridX <= endGridX; ++gridX) {
            for (int32 gridY = startGridY; gridY <= endGridY; ++gridY) {
                int32 hashIdx = computeHashIndex(gridX, gridY, level);
                if (gridFlag.test(hashIdx)) continue;
                for (ftElem* elem = m_elemBucket[hashIdx]; elem != nullptr; elem = elem->next) {
                    results->push(elem->userdata);
                }
                gridFlag.on(hashIdx);
            }
        }
    }

    gridFlag.cleanup();

}

void ftHierarchicalGrid::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {

    ftBitSet bitMask;
    bitMask.init(m_bucketCapacity);

    for (uint32 i = 0; i < m_elemList.getSize(); ++i) {
        bitMask.clear();

        ftElem* elem = &m_elemList[i];

        if (elem->userdata == nullptr) continue;

        uint32 level = elem->level;
        real size = m_cellSizeTable[level];

        // check current level;
        {
            // check current cell;
            for(ftElem* other = elem->next; other != nullptr; other = other->next) {
                if (elem->aabb.overlap(other->aabb)) {
                    uint32 index = pairs->push();
                    (*pairs)[index].userdataA = elem->userdata;
                    (*pairs)[index].userdataB = other->userdata;
                }
            }
            bitMask.on(elem->bucketIndex);

            int32 gridX = ftFloor(elem->aabb.min.x / size);
            int32 gridY = ftFloor(elem->aabb.min.y / size);

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

        while (level < m_nLevel) {
            size = m_cellSizeTable[level];

            if (m_nObject[level] == 0) {
                ++level;
                continue;
            }

            int32 gridX = ftFloor(elem->aabb.min.x / size);
            int32 gridY = ftFloor(elem->aabb.min.y / size);

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

    bitMask.cleanup();

}

void ftHierarchicalGrid::addCollidingPairInBucket(ftElem *elem, uint32 bucketIndex,
                                                  ftChunkArray<ftBroadPhasePair> *pairs) {
    for (ftElem* other = m_elemBucket[bucketIndex]; other != nullptr; other = other->next) {
        if (elem->aabb.overlap(other->aabb)) {
            uint32 index = pairs->push();
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
    n = n % m_bucketCapacity;
    return n;

}

void ftHierarchicalGrid::insertElemToBucket(ftElem *elem, uint32 bucketIndex) {
    elem->bucketIndex = bucketIndex;
    elem->prev = nullptr;
    elem->next = m_elemBucket[bucketIndex];
    if (m_elemBucket[bucketIndex] != nullptr) m_elemBucket[bucketIndex]->prev = elem;
    m_elemBucket[bucketIndex] =  elem;
}


int ftHierarchicalGrid::getMemoryUsage() {
    return (m_elemList.getSize() * sizeof(ftElem)) + (m_bucketCapacity * sizeof(ftElem*));
}