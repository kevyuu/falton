//
// Created by Kevin Yu on 12/24/15.
//

#ifndef FALTON_CHUNKARRAY_H
#define FALTON_CHUNKARRAY_H

template <typename T>
class ChunkArray {
public:

    ChunkArray(int chunkSize);
    ~ChunkArray();

    T& operator[](const unsigned int idx);

    void addChunk();

    int getCapacity();

private:

    int chunkSize;
    int nChunk;
    int capacity;
    T* objects[100];

};


template <typename T>
ChunkArray<T>::ChunkArray(int chunkSize) {
    this->chunkSize = chunkSize;
    this->nChunk = 1;
    this->capacity = chunkSize;
    objects[0] = new T[chunkSize];
}

template <typename T>
ChunkArray<T>::~ChunkArray() {
    for (int i=0;i<nChunk;i++) {
        delete objects[i];
    }
}

template <typename T>
void ChunkArray<T>::addChunk() {
    objects[nChunk] = new T[chunkSize];
    nChunk++;
    capacity += chunkSize;
}

template <typename T>
T& ChunkArray<T>::operator[] (const unsigned int index) {
    int chunkIndex = index / chunkSize;
    int locationInChunk = index % chunkSize;

    return objects[chunkIndex][locationInChunk];
}

template <typename T>
int ChunkArray<T>::getCapacity() {
    return capacity;
}


#endif //FALTON_CHUNKARRAY_H
