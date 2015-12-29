//
// Created by Kevin Yu on 12/12/15.
//

#include "falton/physics/ftCollider.h"
#include "falton/physics/ftBody.h"

ftColliderTable::ftColliderTable() {
    for (int i=0;i<100;i++) {
        bodyToColliderMap[i] = NULL;
    }
}

ftColliderTable::~ftColliderTable() {

}

ftColliderHandle ftColliderTable::create(const ftColliderDef& colliderDef) {
    ftCollider *collider = new ftCollider;
    collider->restitution = colliderDef.restitution;
    collider->friction = colliderDef.friction;

    ftColliderHandle colliderHandle;
    colliderHandle.id = collider;
}

void ftColliderTable::destroy(ftColliderHandle colHandle) {
    delete colHandle.id;
}

ftColliderHandle ftColliderTable::addCollider(const ftBodyHandle bodyHandle, const ftColliderDef &colliderDef) {
    ftColliderHandle colliderHandle = create(colliderDef);
    colliderHandle.id->next = bodyToColliderMap[bodyHandle.id];
    bodyToColliderMap[bodyHandle.id] = colliderHandle;
    return colliderHandle;
}

void ftColliderTable::setRestitution(ftColliderHandle colHandle, real newRestitution) {
    colHandle.id->restitution = newRestitution;
}

void ftColliderTable::setFriction(ftColliderHandle colHandle, real newFriction) {
    colHandle.id->friction = newFriction;
}

void ftColliderTable::setShape(ftColliderHandle colHandle, ftShapeHandle shapeHandle) {
    colHandle.id->shape = shapeHandle;
}

real ftColliderTable::getRestitution(ftColliderHandle colHandle) {
    return colHandle.id->restitution;
}

real ftColliderTable::getFriction(ftColliderHandle colHandle) {
    return colHandle.id->friction;
}

ftShapeHandle ftColliderTable::getShape(ftColliderHandle colHandle) {
    return colHandle.id->shape;
}

ftColliderHandle ftColliderTable::getColliders(ftBodyHandle bodyHandle) {
    return bodyToColliderMap[bodyHandle.id];
}


ftColliderHandle ftColliderTable::next(ftColliderHandle colliderHandle) {
    return colliderHandle.id->next;
}

