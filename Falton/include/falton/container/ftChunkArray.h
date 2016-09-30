//
// Created by Kevin Yu on 12/24/15.
//

#ifndef FALTON_CHUNKARRAY_H
#define FALTON_CHUNKARRAY_H

#include <cstring>
#include <falton/setting.h>

template <typename T>
class ftChunkArray {
public:

    ftChunkArray();

    void init(int chunkSize);
    void cleanup();

    T& operator[](const uint32 idx) const;

    void addChunk();
    void push(T obj);
    uint32 push();
    void reserve(uint32 size);
    void remove();
    void removeAll();

    uint32 getCapacity() const;
    uint32 getSize() const;

private:

    uint32 chunkSize;
    uint32 nChunk;
    uint32 nObject;
    uint32 capacity;
    uint32 chunkCapacity;

    //make object size dynamic
    T** objects;

};

template <typename T>
ftChunkArray<T>::ftChunkArray() {
    this->chunkSize = 0;
    this->nChunk = 0;
    this->capacity = 0;
    nObject = 0;
}

template <typename T>
void ftChunkArray<T>::init(int chunkSize) {
    this->chunkSize = chunkSize;
    this->nChunk = 1;
    this->capacity = chunkSize;
    this->chunkCapacity = 100;

    objects = new T*[chunkCapacity];
    memset(objects,0,sizeof(T*) * chunkCapacity);

    objects[0] = new T[chunkSize];
    nObject = 0;
}

template <typename T>
void ftChunkArray<T>::cleanup() {
    for (uint32 i=0;i<nChunk;i++) {
        if (objects[i] != nullptr) {
            delete[] objects[i];
        }
    }
    nObject = 0;
    delete[] objects;
}

template <typename T>
void ftChunkArray<T>::addChunk() {
    if (nChunk == chunkCapacity) {
        T** oldChunks = objects;
        objects = new T*[chunkCapacity * 2];
        memcpy(objects, oldChunks, chunkCapacity * sizeof(T*));
        chunkCapacity *= 2;
        delete[] oldChunks;
    }
    objects[nChunk] = new T[chunkSize];
    nChunk++;
    capacity += chunkSize;
}

template <typename T>
void ftChunkArray<T>::push(T obj) {
    if (nObject == capacity) addChunk();
    ++nObject;
    operator[](nObject - 1) = obj;
}


template <typename T>
T& ftChunkArray<T>::operator[] (uint32 index) const {
    ftAssert( index < nObject, "index : "<<index<<" nObject: "<<nObject);
    uint32 chunkIndex = index / chunkSize;
    uint32 locationInChunk = index % chunkSize;

    return objects[chunkIndex][locationInChunk];
}

template <typename T>
uint32 ftChunkArray<T>::push() {
    if (nObject == capacity) addChunk();
    ++nObject;
    ftAssert(nObject <= chunkSize * nChunk, "nObject : "<<nObject<<", chunkSize : "<<chunkSize);
    return nObject - 1;
}

template <typename T>
void ftChunkArray<T>::remove() {
    --nObject;
}

template <typename T>
void ftChunkArray<T>::removeAll() {
    nObject = 0;
}

template <typename T>
uint32 ftChunkArray<T>::getCapacity() const{
    return capacity;
}

template <typename T>
uint32 ftChunkArray<T>::getSize() const{
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
