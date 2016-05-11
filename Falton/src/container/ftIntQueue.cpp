//
// Created by Kevin Yu on 3/12/16.
//

#include "falton/container/ftIntQueue.h"
#include <string.h>

void ftIntQueue::init(){
    this->front = 0;
    this->end = -1;
    this->size = 0;

    this->capacity = 64;
    this->handle = new uint32[this->capacity];
}

void ftIntQueue::cleanup() {
    delete this->handle;
}

void ftIntQueue::push(uint32 handle) {
    if (this->size == this->capacity) {
        uint32 *oldHandle = this->handle;
        this->handle = new uint32[this->capacity * 2];
        memcpy(this->handle,oldHandle, this->capacity * sizeof(int));
        this->capacity *= 2;

        delete oldHandle;
    }

    this->end += 1;
    if (this->end == this->capacity) this->end = 0;
    this->handle[this->end] = handle;
}

uint32 ftIntQueue::pop() {
    uint32 frontValue = this->handle[this->front];
    this->front += 1;
    if (this->front == this->capacity) this->front = 0;
    return frontValue;
}

uint32 ftIntQueue::getSize() {
    return size;
}
