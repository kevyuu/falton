//
// Created by Kevin Yu on 12/6/15.
//

#include <falton/physics/ftCollider.h>

ftBody::ftBody(const ftBodyDef& bodyDef) {
    init(bodyDef);
}

void ftBody::init(const ftBodyDef &bodyDef) {

    transform.center = bodyDef.position;
    transform.rotation.setAngle(bodyDef.orientation);

    velocity = bodyDef.velocity;

    centerOfMass = bodyDef.centerOfMass;

    angularVelocity = bodyDef.angularVelocity;

    bodyType = bodyDef.bodyType;

    if (bodyType == DYNAMIC) {
        mass = bodyDef.mass;
        inverseMass = 1 / mass;
        moment = bodyDef.moment;
        inverseMoment = 1 / moment;
    } else {
        mass = real_Infinity;
        inverseMass = 0;
        moment = real_Infinity;
        inverseMoment = 0;
    }

    prev = nullptr;
    next = nullptr;
    colliders = nullptr;

}

void ftBody::addCollider(ftCollider *collider) {
    collider->next = colliders;
    colliders = collider;
}