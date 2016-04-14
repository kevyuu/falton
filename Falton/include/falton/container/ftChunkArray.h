//
// Created by Kevin Yu on 12/24/15.
//

#ifndef FALTON_CHUNKARRAY_H
#define FALTON_CHUNKARRAY_H

#include <string>
#include "falton/math/type.h"

template <typename T>
class ftChunkArray {
public:

    ftChunkArray(int chunkSize);
    ~ftChunkArray();

    T& operator[](const uint32 idx);

    void addChunk();
    void push(T obj);
    uint32 add();
    void reserve(uint32 size);
    void remove();

    uint32 getCapacity();

    uint32 getSize();

private:

    uint32 chunkSize;
    uint32 nChunk;
    uint32 nObject;
    uint32 capacity;
    T* objects[100];

};


template <typename T>
ftChunkArray<T>::ftChunkArray(int chunkSize) {
    this->chunkSize = chunkSize;
    this->nChunk = 1;
    this->capacity = chunkSize;

    memset(objects,0,sizeof(T*) * 100);

    objects[0] = new T[chunkSize];
    nObject = 0;

}

template <typename T>
ftChunkArray<T>::~ftChunkArray() {
    for (uint32 i=0;i<nChunk;i++) {
        delete[] objects[i];
    }
}

template <typename T>
void ftChunkArray<T>::addChunk() {
    objects[nChunk] = new T[chunkSize];
    nChunk++;
    capacity += chunkSize;
}

template <typename T>
void ftChunkArray<T>::push(T obj) {
    if (nObject == capacity) addChunk();
    operator[](nObject) = obj;
    nObject++;
}


template <typename T>
T& ftChunkArray<T>::operator[] (uint32 index) {
    uint32 chunkIndex = index / chunkSize;
    uint32 locationInChunk = index % chunkSize;

    return objects[chunkIndex][locationInChunk];
}

template <typename T>
uint32 ftChunkArray<T>::add() {
    if (nObject == capacity) addChunk();
    nObject++;
    return nObject - 1;
}

template <typename T>
void ftChunkArray<T>::remove() {
    nObject--;
}

template <typename T>
uint32 ftChunkArray<T>::getCapacity() {
    return capacity;
}

template <typename T>
uint32 ftChunkArray<T>::getSize() {
    return nObject;
}

template <typename T>
void ftChunkArray<T>::reserve(uint32 size) {
    if (size > nObject) nObject = size;
    int sizeToAllocate = size - this->capacity;
    if (sizeToAllocate <= 0) return;
    int chunkToAdd = (sizeToAllocate / this->nChunk) + 1;
    for (int i=0;i<chunkToAdd;++i) {
        addChunk();
    }
}

#endif //FALTON_CHUNKARRAY_H
