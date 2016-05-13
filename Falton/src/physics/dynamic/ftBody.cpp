//
// Created by Kevin Yu on 12/6/15.
//

#include <falton/physics/dynamic/ftCollider.h>
#include <falton/physics/dynamic/ftBody.h>

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