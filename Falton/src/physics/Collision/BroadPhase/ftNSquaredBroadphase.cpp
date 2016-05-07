//
// Created by Kevin Yu on 12/30/15.
//

#include <falton/physics/collision/ftCollisionSystem.h>
#include "falton/physics/shape/ftShape.h"
#include "falton/physics/Collision/BroadPhase/ftNSquaredBroadphase.h"

void ftNSquaredBroadphase::init() {
    elements = new ftChunkArray<ftElem>(64);
    nShape = 0;
}

void ftNSquaredBroadphase::shutdown() {
    delete elements;
}

ftBroadphaseHandle ftNSquaredBroadphase::addShape(const ftCollisionShape* colShape, const void* userData) {
    ftBroadphaseHandle freeHandle = this->nShape;
    if (freeHandleList.getSize() > 0) {
        freeHandle = freeHandleList.pop();
    } else {
        freeHandle = elements->add();
    }

    ftElem* elem = &(*this->elements)[freeHandle];

    elem->aabb = colShape->shape->constructAABB(colShape->transform);
    elem->collisionShape = colShape;
    elem->userdata = userData;

    this->nShape++;

    return freeHandle;

}

void ftNSquaredBroadphase::removeShape(ftBroadphaseHandle handle) {
    ftElem* elem = &(*this->elements)[handle];
    elem->collisionShape = nullptr;
    freeHandleList.push(handle);
}

void ftNSquaredBroadphase::moveShape(ftBroadphaseHandle handle) {
    ftElem* elem = &(*this->elements)[handle];
    elem->aabb = elem->collisionShape->shape->constructAABB(elem->collisionShape->transform);
}

void ftNSquaredBroadphase::findPairs(ftChunkArray<ftBroadPhasePair> *pairs) {

    ftBroadPhasePair pair;
    for (uint32 i=0;i< nShape;i++) {
        for (uint32 j = i+1; j < nShape; j++) {
            ftElem* elem1 = &(*this->elements)[i];
            ftElem* elem2 = &(*this->elements)[j];
            if (elem1->collisionShape != nullptr && elem2->collisionShape != nullptr &&
                    elem1->aabb.overlap(elem2->aabb)) {
                pair.userdataA = elem1->userdata;
                pair.userdataB = elem2->userdata;
                pairs.push(pair);
            }
        }
    }
}