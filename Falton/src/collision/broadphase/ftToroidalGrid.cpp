//
// Created by Kevin Yu on 2016-05-21.
//

#include <falton/collision/ftCollisionSystem.h>
#include <falton/collision/broadphase/ftToroidalGrid.h>

const int32 ftToroidalGrid::NULL_NODE = -1;

void ftToroidalGrid::setConfiguration(const ftConfig &config)
{
    m_config = config;
}

void ftToroidalGrid::init()
{
    m_elements.init(64);
    m_free = NULL_NODE;

    m_nBucket = m_config.nRow * m_config.nCol;
    m_buckets = new ftChunkArray<int32>[ m_nBucket ];

    for (int32 i = 0; i < m_nBucket; ++i)
    {
        m_buckets[i].init(16);
    }
    m_bucketMask.init(m_nBucket);
}

void ftToroidalGrid::shutdown()
{
    m_elements.cleanup();
    m_free = NULL_NODE;

    for (int32 i = 0; i < m_nBucket; ++i)
    {
        m_buckets[i].cleanup();
    }

    delete[] m_buckets;
    m_bucketMask.cleanup();
}

ftBroadphaseHandle ftToroidalGrid::addShape(const ftShape *shape, const ftTransform &transform, const void *const userData)
{
    int32 handle;
    if (m_free != NULL_NODE)
    {
        handle = m_free;
        m_free = m_elements[m_free].next;
    }
    else
    {
        handle = m_elements.push();
    }

    m_elements[handle].userdata = userData;
    m_elements[handle].aabb = shape->constructAABB(transform);
    real minX = m_elements[handle].aabb.min.x;
    real minY = m_elements[handle].aabb.min.y;
    real maxX = m_elements[handle].aabb.max.x;
    real maxY = m_elements[handle].aabb.max.y;

    m_elements[handle].xStart = ftFloor(minX / m_config.cellSize);
    m_elements[handle].yStart = ftFloor(minY / m_config.cellSize);
    m_elements[handle].xEnd = ftCeil(maxX / m_config.cellSize) - 1;
    m_elements[handle].yEnd = ftCeil(maxY / m_config.cellSize) - 1;

    insertToBucket(handle);

    return handle;
}

void ftToroidalGrid::removeShape(ftBroadphaseHandle handle)
{

    removeFromBucket(handle);

    m_elements[handle].next = m_free;
    m_free = handle;
}

void ftToroidalGrid::moveShape(ftBroadphaseHandle handle, const ftShape *shape, const ftTransform &transform)
{

    int32 oldXStart = ftFloor(m_elements[handle].aabb.min.x / m_config.cellSize);
    int32 oldYStart = ftFloor(m_elements[handle].aabb.min.y / m_config.cellSize);
    int32 oldXEnd = ftCeil(m_elements[handle].aabb.max.x / m_config.cellSize) - 1;
    int32 oldYEnd = ftCeil(m_elements[handle].aabb.max.y / m_config.cellSize) - 1;

    m_elements[handle].aabb = shape->constructAABB(transform);
    real minX = m_elements[handle].aabb.min.x;
    real minY = m_elements[handle].aabb.min.y;
    real maxX = m_elements[handle].aabb.max.x;
    real maxY = m_elements[handle].aabb.max.y;

    int32 newXStart = ftFloor(minX / m_config.cellSize);
    int32 newYStart = ftFloor(minY / m_config.cellSize);
    int32 newXEnd = ftCeil(maxX / m_config.cellSize) - 1;
    int32 newYEnd = ftCeil(maxY / m_config.cellSize) - 1;

    if (oldXStart != newXStart || oldYStart != newYStart ||
        oldXEnd != newXEnd || oldYEnd != newYEnd)
    {
        removeFromBucket(handle);
        m_elements[handle].xStart = newXStart;
        m_elements[handle].yStart = newYStart;
        m_elements[handle].xEnd = newXEnd;
        m_elements[handle].yEnd = newYEnd;
        insertToBucket(handle);
    }
}

void ftToroidalGrid::findPairs(ftChunkArray<ftBroadPhasePair> *pairs)
{
    for (int32 i = 0; i < m_nBucket; ++i)
    {
        int32 nObjectInBucket = m_buckets[i].getSize();

        for (int32 j = 0; j < nObjectInBucket - 1; ++j)
        {
            for (int32 k = j + 1; k < nObjectInBucket; ++k)
            {

                int32 handleA = m_buckets[i][j];
                int32 handleB = m_buckets[i][k];
                if (m_elements[handleA].aabb.overlap(m_elements[handleB].aabb))
                {
                    int32 index = pairs->push();
                    (*pairs)[index].userdataA = m_elements[handleA].userdata;
                    (*pairs)[index].userdataB = m_elements[handleB].userdata;
                }
            }
        }
    }
}

void ftToroidalGrid::regionQuery(const ftAABB &region, ftChunkArray<const void *> *results)
{
    int32 xStart = ftFloor(region.min.x / m_config.cellSize);
    int32 yStart = ftFloor(region.min.y / m_config.cellSize);
    int32 xEnd = ftCeil(region.max.x / m_config.cellSize) - 1;
    int32 yEnd = ftCeil(region.max.y / m_config.cellSize) - 1;

    if (xEnd - xStart >= m_config.nCol)
    {
        xStart = 0;
        xEnd = m_config.nCol - 1;
    }
    if (yEnd - yStart >= m_config.nRow)
    {
        yStart = 0;
        yEnd = m_config.nRow - 1;
    }

    for (int32 i = yStart; i <= yEnd; ++i)
    {
        for (int32 j = xStart; j <= xEnd; ++j)
        {
            int32 yGrid = ftPositiveMod(i, m_config.nRow);
            int32 xGrid = ftPositiveMod(j, m_config.nCol);
            int32 bucketIndex = yGrid * m_config.nCol + xGrid;
            int32 nObjectInBucket = m_buckets[bucketIndex].getSize();
            for (int32 k = 0; k < nObjectInBucket; ++k)
            {
                int32 handle = m_buckets[bucketIndex][k];
                if (region.overlap(m_elements[handle].aabb))
                    results->push(m_elements[handle].userdata);
            }
        }
    }
}

void ftToroidalGrid::insertToBucket(ftBroadphaseHandle handle)
{
    int32 xStart = m_elements[handle].xStart;
    int32 yStart = m_elements[handle].yStart;
    int32 xEnd = m_elements[handle].xEnd;
    int32 yEnd = m_elements[handle].yEnd;

    if (xEnd - xStart >= m_config.nCol)
    {
        xStart = 0;
        xEnd = m_config.nCol - 1;
    }
    if (yEnd - yStart >= m_config.nRow)
    {
        yStart = 0;
        yEnd = m_config.nRow - 1;
    }

    for (int32 i = yStart; i <= yEnd; ++i)
    {
        for (int32 j = xStart; j <= xEnd; ++j)
        {
            int32 yGrid = ftPositiveMod(i, m_config.nRow);
            int32 xGrid = ftPositiveMod(j, m_config.nCol);
            int32 bucketIndex = yGrid * m_config.nCol + xGrid;
            m_buckets[bucketIndex].push(handle);
        }
    }
}

void ftToroidalGrid::removeFromBucket(ftBroadphaseHandle handle)
{

    int32 xStart = m_elements[handle].xStart;
    int32 yStart = m_elements[handle].yStart;
    int32 xEnd = m_elements[handle].xEnd;
    int32 yEnd = m_elements[handle].yEnd;

    if (xEnd - xStart >= m_config.nCol)
    {
        xStart = 0;
        xEnd = m_config.nCol - 1;
    }
    if (yEnd - yStart >= m_config.nRow)
    {
        yStart = 0;
        yEnd = m_config.nRow - 1;
    }

    for (int32 i = yStart; i <= yEnd; ++i)
    {
        for (int32 j = xStart; j <= xEnd; ++j)
        {
            int32 yGrid = ftPositiveMod(i, m_config.nRow);
            int32 xGrid = ftPositiveMod(j, m_config.nCol);
            int32 bucketIndex = yGrid * m_config.nCol + xGrid;

            int32 nObject = m_buckets[bucketIndex].getSize();
            int32 lastHandle = m_buckets[bucketIndex][nObject - 1];

            for (int32 k = 0; k < nObject; ++k)
            {
                if (m_buckets[bucketIndex][k] == handle)
                {
                    m_buckets[bucketIndex][k] = lastHandle;
                    m_buckets[bucketIndex].remove();
                    break;
                }
            }
        }
    }
}

int ftToroidalGrid::getMemoryUsage()
{
    return (m_elements.getSize() * sizeof(ftElem)) + (m_nBucket * sizeof(int32));
}