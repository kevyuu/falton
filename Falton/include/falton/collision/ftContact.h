//
// Created by Kevin Yu on 4/15/16.
//

#ifndef FALTON_FTCONTACT_H
#define FALTON_FTCONTACT_H

#include <cstdint>
#include <falton/math/math.h>
#include <falton/container/ftChunkArray.h>
#include <falton/container/ftRHHashTable.h>

#include <iostream>
using namespace std;

class ftShape;
struct ftContactEdge;
struct ftCollisionShape;

typedef uint32 ftColHandle;

struct ftManifoldPoint {

    ftVector2 r1; //first shape contact point
    ftVector2 r2; //second shape contact point

    real nIAcc = 0.0;
    real tIAcc = 0.0;
};

struct ftManifold {
    ftVector2 normal; //normal pointing from first shape to second shape
    ftManifoldPoint contactPoints[2];
    real penetrationDepth[2];
    uint8 numContact = 0;
};

typedef enum ftCollisionState {
    BEGIN_COLLISION,
    IN_COLLISION,
    END_COLLISION
} ftCollisionState;

struct ftContact {

    void *userdataA, *userdataB;

    ftManifold manifold;

    ftCollisionState collisionState;

    uint8 timestamp;

    uint32 islandIndex; // used by ftIslandSystem to locate ftContact position

};

class ftContactBuffer {

private:

    struct ftContactElem {
        ftContact contact;
        uint32 hashValue;
        ftColHandle handleA, handleB;
        ftContactElem *next;
        ftContactElem *prev;
    };

    ftContactElem **m_hashTable;
    uint32 m_capacity;
    uint32 m_nContact;

    ftChunkArray<ftContactElem> m_elements;
    ftContactElem* m_freeElementHead;

    uint32 hash(ftColHandle keyA, ftColHandle keyB, uint32 capacity);
    uint64 pairingFunction(ftColHandle keyA, ftColHandle keyB);
    void rehash(uint32 newCapacity);
    void add(ftContactElem* elem, uint32 hashVal);
    ftContactElem* allocateElement();
    void freeElement(ftContactElem* elem);

public:

    void init();
    void cleanup();

    ftContact* find(ftColHandle  key1, ftColHandle key2);
    ftContact* create(ftColHandle key1, ftColHandle key2);
    void destroy(ftContact* contact);

    uint32 getSize();

    /* template IteratorT {
     *      void operator()(int32 handleA, int32 handleB, ftContact* contact);
     * } */
    template <typename IteratorT>
    void forEach(IteratorT func);

    /* template IteratorT {
     *      bool operator()(int32 handleA, int32 handleB, ftContact* contact);
     * } */
    template <typename IteratorT>
    void removeIf(IteratorT func);

};

template <typename IteratorT>
void ftContactBuffer::forEach(IteratorT func) {
    for (uint32 i = 0; i < m_elements.getSize(); ++i) {
        if (m_elements[i].hashValue != nulluint) {
            ftColHandle &handleA = m_elements[i].handleA;
            ftColHandle &handleB = m_elements[i].handleB;
            ftContact* contact = &m_elements[i].contact;
            func(handleA, handleB, contact);
        }
    }
}

template <typename IteratorT>
void ftContactBuffer::removeIf(IteratorT func) {
    for (uint32 i = 0; i < m_elements.getSize(); ++i) {
        ftColHandle &handleA = m_elements[i].handleA;
        ftColHandle &handleB = m_elements[i].handleB;
        ftContact* contact = &m_elements[i].contact;
        if (m_elements[i].hashValue != nulluint && func(handleA, handleB, contact)) {
            destroy(contact);
        }
    }
}

class ftContactBuffer3 {
private:
    ftRHHashTable hashTable;
    struct ftContactElem {
        ftContact contact;
        union {
            ftColHandle handleA;
            uint32 isAllocated;
        };
        union {
            ftColHandle handleB;
            ftContactElem* next;
        };
    };

    ftChunkArray<ftContactElem> m_elements;
    ftContactElem* m_freeElementHead;

    ftContactElem* allocateElement();
    void freeElement(ftContactElem* element);

public:

    void init();
    void cleanup();

    ftContact* find(ftColHandle  key1, ftColHandle key2);
    ftContact* create(ftColHandle key1, ftColHandle key2);
    void destroy(ftContact* contact);

    uint32 getSize();

    /* template IteratorT {
     *      void operator()(int32 handleA, int32 handleB, ftContact* contact);
     * } */
    template <typename IteratorT>
    void forEach(IteratorT func);

    /* template IteratorT {
     *      bool operator()(int32 handleA, int32 handleB, ftContact* contact);
     * } */
    template <typename IteratorT>
    void removeIf(IteratorT func);

};

template <typename IteratorT>
void ftContactBuffer3::forEach(IteratorT func) {
    for (uint32 i = 0 ; i < m_elements.getSize(); ++i) {
        if (m_elements[i].isAllocated != nulluint) {
            func(m_elements[i].handleA, m_elements[i].handleB, &m_elements[i].contact);
        }
    }
}

template <typename IteratorT>
void ftContactBuffer3::removeIf(IteratorT func) {
    for (uint32 i = 0 ; i < m_elements.getSize(); ++i) {
        if (m_elements[i].isAllocated != -1 &&
                func(m_elements[i].handleA, m_elements[i].handleB, &m_elements[i].contact)) {
            destroy(&m_elements[i].contact);
        }
    }
}



#endif //FALTON_FTCONTACT_H
