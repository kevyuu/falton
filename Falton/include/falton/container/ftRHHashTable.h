//
// Created by Kevin Yu on 2016-05-24.
//

#ifndef FALTON_RHHASHTABLE_H
#define FALTON_RHHASHTABLE_H

#include <falton/setting/general.h>

class ftRHHashTable {
public:
    void init(int32 capacity);
    void cleanup();
    void insert(int32 key, void *obj);
    void* find(int32 key) const;
    void remove(int32 key);

private:
    struct ftBucket {
        int32 key;
        int32 hashValue;
        void* object;
    };

    ftBucket* m_buckets;
    int32 m_capacity;
    int32 m_objectLimit;

    int32 m_nObject;
    int32 m_maxDIB;

    int32 hash(int32 key) const;
    void growBucket();

    void insertToBucket(int32 key, void* obj);

};


#endif //FALTON_RHHASHTABLE_H
