//
// Created by Kevin Yu on 2016-05-21.
//

#include <falton/physics/collision/ftCollisionSystem.h>
#include "falton/physics/collision/broadphase/ftToroidalGrid.h"

#include <iostream>
using namespace std;

const int32 ftToroidalGrid::NULL_NODE = -1;

void ftToroidalGrid::setConfiguration(const ftConfig& config) {
    m_config = config;
}

void ftToroidalGrid::init() {
    m_elements.init(64);
    m_free = NULL_NODE;

    m_nBucket = m_config.nRow * m_config.nCol;
    m_buckets = new ftChunkArray<int>[m_nBucket];

    for (uint32 i = 0 ; i < m_nBucket; ++i) {
        m_buckets[i].init(16);
    }
}

void ftToroidalGrid::shutdown() {
    m_elements.cleanup();
    m_free = NULL_NODE;

    for (uint32 i = 0 ; i < m_nBucket; ++i) {
        m_buckets[i].cleanup();
    }

    delete[] m_buckets;
}

ftBroadphaseHandle ftToroidalGrid::addShape(const ftCollisionShape *const colShape, const void *const userData) {
    int32 handle;
    if (m_free != NULL_NODE) {
        handle = m_free;
        m_free = m_elements[m_free].next;
    } else {
        handle = m_elements.add();
    }

    m_elements[handle].userdata = userData;
    m_elements[handle].aabb = colShape->shape->constructAABB(colShape->transform);
    real minX = m_elements[handle].aabb.min.x;
    real minY = m_elements[handle].aabb.min.y;
    real maxX = m_elements[handle].aabb.max.x;
    real maxY = m_elements[handle].aabb.max.y;

    m_elements[handle].xStart = minX / m_config.cellSize;
    m_elements[handle].yStart = minY / m_config.cellSize;
    m_elements[handle].xEnd = ftCeil(maxX / m_config.cellSize);
    m_elements[handle].yEnd = ftCeil(maxY / m_config.cellSize);

    insertToBucket(handle);

    return handle;
}

void ftToroidalGrid::removeShape(ftBroadphaseHandle handle) {

    removeFromBucket(handle);

    m_elements[handle].next = m_free;
    m_free = handle;

}

void ftToroidalGrid::moveShape(ftBroadphaseHandle handle, const ftCollisionShape &collisionShape) {

    removeFromBucket(handle);

    m_elements[handle].aabb = collisionShape.shape->constructAABB(collisionShape.transform);
    real minX = m_elements[handle].aabb.min.x;
    real minY = m_elements[handle].aabb.min.y;
    real maxX = m_elements[handle].aabb.max.x;
    real maxY = m_elements[handle].aabb.max.y;

    m_elements[handle].xStart = ftFloor(minX / m_config.cellSize);
    m_elements[handle].yStart = ftFloor(minY / m_config.cellSize);
    m_elements[handle].xEnd = ftCeil(maxX / m_config.cellSize);
    m_elements[handle].yEnd = ftCeil(maxY / m_config.cellSize);

    insertToBucket(handle);

}

void ftToroidalGrid::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {
    for (uint32 i = 0; i < m_nBucket; ++i) {
        int32 nObjectInBucket = m_buckets[i].getSize();

        for (int32 j = 0; j < nObjectInBucket - 1; ++j) {
            for (int32 k = j+1; k < nObjectInBucket; ++k) {

                int32 handleA = m_buckets[i][j];
                int32 handleB = m_buckets[i][k];
                if (handleA != handleB && m_elements[handleA].aabb.overlap(m_elements[handleB].aabb)) {
                    int32 index = pairs->add();
                    (*pairs)[index].userdataA = m_elements[handleA].userdata;
                    (*pairs)[index].userdataB = m_elements[handleB].userdata;
                }

            }
        }

    }
}

void ftToroidalGrid::insertToBucket(ftBroadphaseHandle handle) {
    for (int32 i = m_elements[handle].yStart; i <= m_elements[handle].yEnd; ++i) {
        for (int32 j = m_elements[handle].xStart; j <= m_elements[handle].xEnd; ++j) {
            int32 yGrid = ftPositiveMod(i , m_config.nRow);
            int32 xGrid = ftPositiveMod(j , m_config.nCol);
            int32 bucketIndex = yGrid * m_config.nCol + xGrid;
            m_buckets[bucketIndex].push(handle);
        }
    }
}

void ftToroidalGrid::removeFromBucket(ftBroadphaseHandle handle) {
    for (int32 i = m_elements[handle].yStart; i <= m_elements[handle].yEnd; ++i) {
        for (int32 j = m_elements[handle].xStart; j <= m_elements[handle].xEnd; ++j) {
            int32 yGrid = ftPositiveMod(i , m_config.nRow);
            int32 xGrid = ftPositiveMod(j , m_config.nCol);
            int32 bucketIndex = yGrid * m_config.nCol + xGrid;

            int32 nObject = m_buckets[bucketIndex].getSize();
            int32 lastHandle = m_buckets[bucketIndex][nObject - 1];

            for (uint32 k = 0; k < nObject; ++k) {
                if (m_buckets[bucketIndex][k] == handle) {
                    m_buckets[bucketIndex][k] = lastHandle;
                    m_buckets[bucketIndex].remove();
                    break;
                }
            }

        }
    }
}

