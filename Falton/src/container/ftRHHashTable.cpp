//
// Created by Kevin Yu on 2016-05-24.
//

#include "falton/container/ftRHHashTable.h"

void ftRHHashTable::init(int32 capacity) {
    m_capacity = capacity;
    m_objectLimit = 0.8 * m_capacity;
    m_maxDIB = -1;
    m_buckets = new ftBucket[m_capacity];

}

void ftRHHashTable::insert(int32 key, void* obj) {
    ftAssert(obj != nullptr);
    if (m_nObject > m_objectLimit) {
        growBucket();
    }
    insertToBucket(key,obj);
    ++m_nObject;
}

void* ftRHHashTable::find(int32 key) const {
    int32 hashValue = hash(key);
    int32 index = hashValue;
    while ( m_buckets[index].key != key && m_buckets[index].object != nullptr
            && (index - hashValue) <= m_maxDIB ) {
        ++index;
        index %= m_capacity;
    }
    if ( (index - hashValue) > m_maxDIB ) {
        return nullptr;
    } else {
        return m_buckets[index].object;
    }
}

void ftRHHashTable::remove(int32 key) {
    int32 hashValue = hash(key);
    int32 index = hashValue;
    while ( m_buckets[index].object != nullptr ) {
        int32 nextIndex = index + 1;
        nextIndex %= m_capacity;
        m_buckets[index].key = m_buckets[nextIndex].key;
        m_buckets[index].hashValue = m_buckets[nextIndex].hashValue;
        m_buckets[index].object = m_buckets[nextIndex].object;
        index = nextIndex;
    }
    --m_nObject;
}

int32 ftRHHashTable::hash(int32 key) const {
    key = ((key >> 16) ^ key) * 0x45d9f3b;
    key = ((key >> 16) ^ key) * 0x45d9f3b;
    key = ((key >> 16) ^ key);
    return key % m_capacity;
}

void ftRHHashTable::growBucket() {
    ftBucket* oldBuckets = m_buckets;
    int32 oldCapacity = m_capacity;
    m_capacity *= 2;
    m_objectLimit = 0.8 * m_capacity;
    m_maxDIB = -1;
    m_buckets = new ftBucket[m_capacity];

    for (int32 i = 0 ; i < oldCapacity ; ++i) {
        insertToBucket(m_buckets[i].key, m_buckets[i].object);
    }

    delete[] oldBuckets;
}

void ftRHHashTable::insertToBucket(int32 key, void *obj) {
    int32 hashValue = hash(key);
    int32 index = hashValue;
    while ( m_buckets[index].object != nullptr ) {
        if ( (index - m_buckets[index].hashValue) < (index - hashValue) ) {
            int32 tmpKey = m_buckets[index].key;
            int32 tmpHashValue = m_buckets[index].hashValue;
            void* tmpObj = m_buckets[index].object;

            m_buckets[index].key = key;
            m_buckets[index].hashValue = hashValue;
            m_buckets[index].object = obj;

            key = tmpKey;
            hashValue = tmpHashValue;
            obj = tmpObj;
        }
        ++index;
        index %= m_capacity;
    }

    m_buckets[index].key = key;
    m_buckets[index].hashValue = hashValue;
    m_buckets[index].object = obj;

    if (m_maxDIB < (index - hashValue)) {
        m_maxDIB = index - hashValue;
    }
}

void ftRHHashTable::cleanup() {
    delete[] m_buckets;
}