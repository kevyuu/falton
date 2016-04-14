//
// Created by Kevin Yu on 12/12/15.
//

#include "falton/physics/ftCollider.h"
#include <iostream>

using namespace std;

ftCollider::ftCollider(const ftColliderDef& colliderDef) :
        transform(colliderDef.position, colliderDef.orientation) {

    body = colliderDef.body;

    friction = colliderDef.friction;
    restitution = colliderDef.restitution;

    shape = colliderDef.shape;
}