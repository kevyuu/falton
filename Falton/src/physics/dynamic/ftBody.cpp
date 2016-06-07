//
// Created by Kevin Yu on 12/6/15.
//

#include <falton/physics/dynamic/ftCollider.h>
#include <falton/physics/dynamic/ftBody.h>

void ftBody::applyForce(ftVector2 force, ftVector2 localPos) {
    forceAccum += force;
    ftVector2 r = localPos - centerOfMass;
    torqueAccum += (r.cross(force));
}

void ftBody::applyForceAtCenterOfMass(ftVector2 force) {
    forceAccum += force;
}

void ftBody::applyTorque(real torque) {
    torqueAccum += torque;
}

void ftBody::setMass(float mass) {
    if (bodyType != DYNAMIC) return;
    this->mass = mass;
    if (mass == 0) {
        inverseMass = real_Infinity;
    } else {
        inverseMass = 1/mass;
    }
}

void ftBody::setMoment(real moment) {
    if (bodyType != DYNAMIC) return;
    this->moment = moment;
    if (moment == 0) {
        inverseMoment = real_Infinity;
    } else {
        inverseMoment = 1 / moment;
    }
}

ftBody* ftBodyBuffer::create() {

    ftBodyElem *element = new ftBodyElem;

    element->next = bodies;
    if (bodies!=nullptr) bodies->prev = element;
    bodies = element;

    ++size;

    return (ftBody*) element;
}

void ftBodyBuffer::destroy(ftBody *body) {

    ftBodyElem *element = (ftBodyElem *) body;

    ftBodyElem *next = element->next;
    ftBodyElem *prev = element->prev;

    if (prev!=nullptr) {
        prev->next = next;
    } else {
        bodies = next;
    }

    if (next!=nullptr) next->prev = prev;

    --size;

    delete element;
}


ftBodyBuffer::ftIter ftBodyBuffer::iterate() {
    ftIter iter;
    iter.curElem = bodies;
    return iter;
}

ftBody* ftBodyBuffer::start(ftIter *iter) {
    ftBody *body = (ftBody*) iter->curElem;
    return body;
}

ftBody* ftBodyBuffer::next(ftIter *iter) {
    iter->curElem = iter->curElem->next;
    ftBody* body = (ftBody*) iter->curElem;
    return body;
}

uint32 ftBodyBuffer::getSize() {
    return size;
}

void ftBodyBuffer::insert(ftBody *body) {

    ftBodyElem* element = (ftBodyElem*) body;
    element->next = bodies;
    element->prev = nullptr;
    if (bodies!=nullptr) bodies->prev = element;
    bodies = element;

    ++size;

}

void ftBodyBuffer::unlink(ftBody *body) {
    ftBodyElem* element = (ftBodyElem*) body;

    ftBodyElem *next = element->next;
    ftBodyElem *prev = element->prev;

    if (prev!=nullptr) {
        prev->next = next;
    } else {
        bodies = next;
    }

    if (next!=nullptr) next->prev = prev;

    element->next = nullptr;
    element->prev = nullptr;

    --size;
}

void ftBodyBuffer::init() {
    bodies = nullptr;
    size = 0;
}

void ftBodyBuffer::cleanup() {
    for (ftBodyElem* element = bodies; element != nullptr;) {
        ftBodyElem* next = element->next;
        delete element;
        element = next;
    }
    bodies = nullptr;
}