//
// Created by Kevin Yu on 12/30/15.
//

#include <falton/physics/collision/ftCollisionSystem.h>
#include "falton/physics/shape/ftShape.h"
#include "falton/physics/Collision/BroadPhase/ftNSquaredBroadphase.h"

void ftNSquaredBroadphase::init() {
    m_elements.init(64);
    m_freeHandleList.init();

    m_nShape = 0;
}

void ftNSquaredBroadphase::shutdown() {
    m_elements.cleanup();
    m_freeHandleList.cleanup();
}

ftBroadphaseHandle ftNSquaredBroadphase::addShape(const ftCollisionShape* colShape, const void* userData) {
    ftBroadphaseHandle freeHandle = this->m_nShape;
    if (m_freeHandleList.getSize() > 0) {
        freeHandle = m_freeHandleList.pop();
    } else {
        freeHandle = m_elements.add();
    }

    ftElem* elem = &m_elements[freeHandle];

    elem->aabb = colShape->shape->constructAABB(colShape->transform);
    elem->collisionShape = colShape;
    elem->userdata = userData;

    this->m_nShape++;

    return freeHandle;

}

void ftNSquaredBroadphase::removeShape(ftBroadphaseHandle handle) {
    ftElem* elem = &m_elements[handle];
    elem->collisionShape = nullptr;
    m_freeHandleList.push(handle);
}

void ftNSquaredBroadphase::moveShape(ftBroadphaseHandle handle, const ftCollisionShape& colShape) {
    ftElem* elem = &m_elements[handle];
    elem->aabb = colShape.shape->constructAABB(colShape.transform);
}

void ftNSquaredBroadphase::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {

    ftBroadPhasePair pair;
    for (uint32 i=0;i< m_nShape;i++) {
        for (uint32 j = i+1; j < m_nShape; j++) {
            ftElem* elem1 = &m_elements[i];
            ftElem* elem2 = &m_elements[j];
            if (elem1->collisionShape != nullptr && elem2->collisionShape != nullptr
                    && elem1->aabb.overlap(elem2->aabb)) {
                pair.userdataA = elem1->userdata;
                pair.userdataB = elem2->userdata;
                pairs->push(pair);
            }
        }
    }
}