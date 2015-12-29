//
// Created by Kevin Yu on 12/18/15.
//

#ifndef FALTON_CHUNKTABLE_H
#define FALTON_CHUNKTABLE_H

#include <deque>


struct Handle {
    int ID;
};

template <typename T>
struct ChunkTableObject {
    T object;
    Handle handle;
};

template <typename T>
class ChunkTable {
private:
    ChunkTableObject<T>* handleMap[];

    ChunkTableObject<T>* objects[10000];

    Handle lastHandle;

    int nObject;

    int chunkSize;

    int capacity;
    int idCapacity;

    int currentChunk;
    int currentIndex;


    std::deque<Handle> freeID;

public:
    ChunkTable(int chunkSize) {
        handleMap = new ChunkTableObject<T>*[chunkSize];

        objects[0] = new ChunkTableObject<T>[chunkSize];

        nObject = 0;

        this->chunkSize = chunkSize;

        capacity = chunkSize;
        idCapacity = chunkSize;

        lastHandle.ID = 0;

        currentChunk = 0;
        currentIndex = 0;

    }

    ~ChunkTable() {
        delete handleMap;
        for (int i=0;i<currentChunk;i++) {
            delete objects[i];
        }
    }

    Handle create() {
        Handle newObjectHandle;
        if (freeID.size() < 1024) {
            if (lastHandle.ID == idCapacity) {
                idCapacity *= 2;
                ChunkTableObject<T>** newHandleMap = new ChunkTableObject<T>*[idCapacity];
                memcpy(newHandleMap, handleMap, lastHandle.ID * sizeof(T*));
                delete handleMap;
                handleMap = newHandleMap;
            }
            newObjectHandle = lastHandle;
            lastHandle.ID++;

        } else {
          newObjectHandle = freeID.front();
        }


        if (nObject == capacity) {
            currentChunk++;
            currentIndex = 0;
            objects[currentChunk] = new ChunkTableObject<T>[chunkSize];
            capacity += chunkSize;
        }

        objects[currentChunk][currentIndex].handle = newObjectHandle;
        handleMap[newObjectHandle.ID] = &(objects[currentChunk][currentIndex]);

        currentIndex++;
        nObject++;
    }

    void destroy(Handle handle) {

        freeID.push_back(handle);
        T* object = getChunkTableObject(handle);
        *object = objects[currentChunk][currentIndex];
        int lastObjectID = objects[currentChunk][currentIndex].handle.ID;

        handleMap[handle.ID] = NULL;
        handleMap[lastObjectID] = object;

        nObject--;

        currentIndex--;

        if (currentIndex == 0) {
            delete object[currentChunk];
            currentChunk--;
            currentIndex = chunkSize;
            capacity = nObject;
        }
    }

    T* get(int index) {
        int chunkIndex = index / chunkSize;
        int positionInChunk = index % chunkSize;
        return &(objects[chunkIndex][positionInChunk].object);
    }

    T* get(Handle handle) {
        return &(handleMap[handle.ID]->object);
    }

    ChunkTableObject<T>* getChunkTableObject(Handle handle) {
        return handleMap[handle.ID];
    }

};



#endif //FALTON_CHUNKTABLE_H
