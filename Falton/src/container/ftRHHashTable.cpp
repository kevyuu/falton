//
// Created by Kevin Yu on 2016-05-24.
//

#include <cstring>
#include <falton/container/ftRHHashTable.h>

void ftRHHashTable::init(int32 capacity) {
    m_capacity = capacity;
    m_objectLimit = 0.8 * m_capacity;
    m_maxDIB = -1;
    m_buckets = new ftBucket[m_capacity];

    memset(m_buckets,0, m_capacity * sizeof(*m_buckets));

    m_nObject = 0;

}

void ftRHHashTable::insert(int32 key1, int32 key2, void* obj) {
    ftAssert(obj != nullptr, "");
    if (m_nObject > m_objectLimit) {
        growBucket();
    }
    insertToBucket(key1, key2, obj);
    ++m_nObject;
}

void ftRHHashTable::growBucket() {
    ftBucket* oldBuckets = m_buckets;
    int32 oldCapacity = m_capacity;
    m_capacity *= 2;
    m_objectLimit = 0.8 * m_capacity;
    m_maxDIB = -1;
    m_buckets = new ftBucket[m_capacity];
    memset(m_buckets,0, m_capacity * sizeof(*m_buckets));

    for (int32 i = 0 ; i < oldCapacity ; ++i) {
        if (oldBuckets[i].object != nullptr) {
            insertToBucket(oldBuckets[i].key1, oldBuckets[i].key2, oldBuckets[i].object);
        }
    }

    delete[] oldBuckets;
}

void ftRHHashTable::insertToBucket(int32 key1, int32 key2, void *obj) {
    int32 hashValue = hash(key1, key2);
    int32 index = hashValue;
    int32 dib = 0;
    while ( m_buckets[index].object != nullptr ) {
        if ( m_buckets[index].dib < dib ) {
            int32 tmpKey1 = m_buckets[index].key1;
            int32 tmpKey2 = m_buckets[index].key2;
            int32 tmpDib = m_buckets[index].dib;
            void* tmpObj = m_buckets[index].object;

            m_buckets[index].key1 = key1;
            m_buckets[index].key2 = key2;
            m_buckets[index].dib = dib;
            m_buckets[index].object = obj;

            key1 = tmpKey1;
            key2 = tmpKey2;
            dib = tmpDib;
            obj = tmpObj;
        }
        ++index;
        ++dib;
        index %= m_capacity;
    }

    m_buckets[index].key1 = key1;
    m_buckets[index].key2 = key2;
    m_buckets[index].dib = dib;
    m_buckets[index].object = obj;

    if (m_maxDIB < dib) {
        m_maxDIB = dib;
    }
}

void* ftRHHashTable::find(int32 key1, int32 key2) const {
    int32 hashValue = hash(key1, key2);
    int32 index = hashValue;
    int32 dib = 0;
    while ((m_buckets[index].key1 != key1 || m_buckets[index].key2 != key2)
           && m_buckets[index].object != nullptr
            && dib <= m_maxDIB ) {
        ++dib;
        ++index;
        index %= m_capacity;
    }
    if ( dib  > m_maxDIB ) {
        return nullptr;
    } else {
        return m_buckets[index].object;
    }
}

void ftRHHashTable::remove(int32 key1, int32 key2) {
    int32 hashValue = hash(key1, key2);
    int32 index = hashValue;
    while ((m_buckets[index].key1 != key1 || m_buckets[index].key2 != key2)) {
        ++index;
        index %= m_capacity;
    }
    removeByIndex(index);
}

void ftRHHashTable::removeByIndex(int32 index) {
    int32 nextIndex = index + 1;
    nextIndex %= m_capacity;
    while ( m_buckets[nextIndex].object != nullptr && m_buckets[nextIndex].dib != 0) {
        m_buckets[index].key1 = m_buckets[nextIndex].key1;
        m_buckets[index].key2 = m_buckets[nextIndex].key2;
        m_buckets[index].dib = m_buckets[nextIndex].dib - 1;
        m_buckets[index].object = m_buckets[nextIndex].object;
        index = nextIndex;
        ++nextIndex;
        nextIndex %= m_capacity;
    }
    m_buckets[index].object = nullptr;
    --m_nObject;
}

int32 ftRHHashTable::hash(int32 key1, int32 key2) const {

    key1 = ((key1 >> 16) ^ key1) * 0x45d9f3b;
    key1 = ((key1 >> 16) ^ key1) * 0x45d9f3b;
    key1 = ((key1 >> 16) ^ key1);
    key2 = ((key2 >> 16) ^ key2) * 0x45d9f3b;
    key2 = ((key2 >> 16) ^ key2) * 0x45d9f3b;
    key2 = ((key2 >> 16) ^ key2);
    int32 key = key1 ^ key2;

    return key % m_capacity;
}

uint32 ftRHHashTable::getSize() {
    return m_nObject;
}

void ftRHHashTable::cleanup() {

    delete[] m_buckets;
}