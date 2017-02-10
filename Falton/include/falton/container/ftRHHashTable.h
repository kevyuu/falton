//
// Created by Kevin Yu on 2016-05-24.
//
#pragma once

#include <falton/setting.h>

class ftRHHashTable {
public:
    void init(int32 capacity);
    void cleanup();
    void insert(int32 key1,int32 key2, void *obj);
    void* find(int32 key1, int32 key2) const;
    void remove(int32 key1, int32 key2);
    void removeByIndex(int32 index);
    uint32 getSize();

    template <typename T>
    void forEach(T func);

    template <typename T>
    void removeIf(T func);

private:
    struct ftBucket {
        int32 key1;
        int32 key2;
        int32 dib;
        void* object;
    };

    ftBucket* m_buckets;
    int32 m_capacity;
    int32 m_objectLimit;

    int32 m_nObject;
    int32 m_maxDIB;

    int32 hash(int32 key1, int32 key2) const;
    void growBucket();

    void insertToBucket(int32 key1, int32 key2, void* obj);

};


template <typename T>
void ftRHHashTable::forEach(T func) {
    for (int32 i = 0 ; i < m_capacity; ++i) {
        if (m_buckets[i].object != nullptr)
            func(m_buckets[i].key1, m_buckets[i].key2, m_buckets[i].object);
    }
}
