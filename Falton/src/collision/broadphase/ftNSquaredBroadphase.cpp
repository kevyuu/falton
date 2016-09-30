//
// Created by Kevin Yu on 12/30/15.
//

#include <falton/collision/ftCollisionSystem.h>
#include <falton/collision/broadphase/ftNSquaredBroadphase.h>

void ftNSquaredBroadphase::init() {
    m_elements.init(64);
    m_freeHandleList.init();

    m_nShape = 0;
}

void ftNSquaredBroadphase::shutdown() {
    m_elements.cleanup();
    m_freeHandleList.cleanup();
}

ftBroadphaseHandle ftNSquaredBroadphase::addShape(const ftShape* shape, const ftTransform& transform, const void* userData) {
    ftBroadphaseHandle freeHandle;
    if (m_freeHandleList.getSize() > 0) {
        freeHandle = m_freeHandleList.pop();
    } else {
        freeHandle = m_elements.push();
    }

    ftElem* elem = &m_elements[freeHandle];

    elem->aabb = shape->constructAABB(transform);
    elem->userdata = userData;
    elem->isAllocated = true;

    ++m_nShape;

    return freeHandle;

}

void ftNSquaredBroadphase::removeShape(ftBroadphaseHandle handle) {
    ftElem* elem = &m_elements[handle];
    elem->isAllocated = false;
    m_freeHandleList.push(handle);
    --m_nShape;
}

void ftNSquaredBroadphase::moveShape(ftBroadphaseHandle handle, const ftShape* shape, const ftTransform& transform) {
    ftElem* elem = &m_elements[handle];
    elem->aabb = shape->constructAABB(transform);
}

void ftNSquaredBroadphase::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {

    ftBroadPhasePair pair;
    for (uint32 i=0;i< m_nShape;++i) {
        for (uint32 j = i+1; j < m_nShape; ++j) {
            ftElem* elem1 = &m_elements[i];
            ftElem* elem2 = &m_elements[j];
            if (elem1->isAllocated && elem2->isAllocated
                    && elem1->aabb.overlap(elem2->aabb)) {
                pair.userdataA = elem1->userdata;
                pair.userdataB = elem2->userdata;
                pairs->push(pair);
            }
        }
    }
}

void ftNSquaredBroadphase::regionQuery(const ftAABB &region, ftChunkArray<const void *> *results) {
    for (uint32 i = 0; i < m_nShape; ++i) {
        if (m_elements[i].isAllocated && region.overlap(m_elements[i].aabb)) {
            results->push(m_elements[i].userdata);
        }
    }
}


int ftNSquaredBroadphase::getMemoryUsage() {
    return m_elements.getSize() * sizeof(ftElem);
}

