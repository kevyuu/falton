#pragma once

template <typename T>
class ftIDBuffer {
public:

    void init(int capacity);
    void cleanup();

    int alloc();
    T* getAddress(int id);
	int getID(T* address);
    T* remove(int id);
	T* remove(T* object);
    void removeAll();

    T* buffer;
    int freeID;
    int* indexMap;
	int* indexBackMap;
    int capacity;
    int size;

    static const int END_FREE_LIST = -1;
    static const int GROW_FACTOR = 2;
};

template <typename T>
void ftIDBuffer<T>::init(int capacity) {
    this->capacity = capacity;
    this->size = 0;
    buffer = (T*) malloc(sizeof(T) * capacity);
    indexMap = (int*) malloc(sizeof(*indexMap) * capacity);
	indexBackMap = (int*)malloc(sizeof(*indexBackMap) * capacity);

    freeID = 0;
    for (int i = 0; i < capacity -1; ++i) {
        indexMap[i] = i + 1;
    }
    indexMap[capacity - 1] = END_FREE_LIST;

}

template <typename T>
int ftIDBuffer<T>::alloc() {
    
    if (size == capacity) {
        int oldCapacity = capacity;
        capacity *= GROW_FACTOR;

        T* oldBuffer = buffer;
        buffer = (T*) malloc(sizeof(T) * capacity);
        memcpy(buffer, oldBuffer, oldCapacity * sizeof(T));
        free(oldBuffer);
        
        int* oldIndexMap = indexMap;
        indexMap = (int*) malloc(sizeof(int) * capacity);
        memcpy(indexMap, oldIndexMap, oldCapacity * sizeof(T));
        free(oldIndexMap);
        freeID = size;
        for (int i = size; i < capacity - 1; ++i) {
            indexMap[i] = i + 1;
        }
        indexMap[capacity - 1] = END_FREE_LIST;
    }

    int objectID = freeID;
    freeID = indexMap[freeID];
    indexMap[objectID] = size;
	indexBackMap[size] = objectID;
    ++size;

    return objectID;
}

template <typename T>
T* ftIDBuffer<T>::getAddress(int id) {
    int internalID = indexMap[id];
    return &(buffer[internalID]);
}

template <typename T>
int ftIDBuffer<T>::getID(T* address) {
	int internalID = (int)(address - buffer);
	return indexBackMap[internalID];
}

template <typename T>
T* ftIDBuffer<T>::remove(int id) {
    int internalID = indexMap[id];
    int lastID = size - 1;
    buffer[internalID] = buffer[lastID];
	int replaceID = indexBackMap[lastID];
	indexMap[replaceID] = internalID;
	indexBackMap[internalID] = replaceID;

	indexMap[id] = freeID;
	freeID = id;

    --size;
    return &(buffer[internalID]);
}

template <typename T>
T* ftIDBuffer<T>::remove(T* object) {
	int internalID = object - buffer;
	int id = indexBackMap[internalID];
	int lastID = size - 1;
	buffer[internalID] = buffer[lastID];
	int replaceID = indexBackMap[lastID];
	indexMap[replaceID] = internalID;
	indexBackMap[internalID] = replaceID;

	indexMap[id] = freeID;
	freeID = id;

	--size;
	return &(buffer[internalID]);
}

template <typename T>
void ftIDBuffer<T>::removeAll() {
    size = 0;
    for (int i = 0; i < capacity - 1; ++i) {
        indexMap[i] = i + 1;
    }
    indexMap[capacity - 1] = END_FREE_LIST;
    freeID = 0;
}

template <typename T>
void ftIDBuffer<T>::cleanup() {
	free(indexBackMap);
    free(indexMap);
    free(buffer);
}

