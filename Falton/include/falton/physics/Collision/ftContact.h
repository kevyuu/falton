//
// Created by Kevin Yu on 4/15/16.
//

#ifndef FALTON_FTCONTACT_H
#define FALTON_FTCONTACT_H

#include <cstdint>
#include <falton/math/math.h>
#include <falton/container/ftChunkArray.h>

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
    int8 numContact = 0;
};

typedef enum ftCollisionState {
    BEGIN_COLLISION,
    IN_COLLISION
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

    ftContactElem **contacts;
    uint32 capacity;
    uint32 nContact;

    uint32 hash(ftColHandle keyA, ftColHandle keyB, uint32 capacity);
    uint64 pairingFunction(ftColHandle keyA, ftColHandle keyB);
    void rehash(uint32 newCapacity);
    void add(ftContactElem* elem, uint32 hashVal);

public:

    void init();
    void cleanup();

    class ftIter {
    public:
        ftColHandle handleA, handleB;
        ftContact* contact;
    private:
        uint32 index;
        friend class ftContactBuffer;
    };

    ftContact* find(ftColHandle  key1, ftColHandle key2);
    ftContact* create(ftColHandle key1, ftColHandle key2);
    void destroy(ftContact* contact);

    uint32 getSize();

    ftIter iterate();
    void next(ftIter* iter);

    template <typename T>
    void forEach(T func);

};

template <typename T>
void ftContactBuffer::forEach(T func) {
    for (uint32 i = 0; i < capacity; ++i) {
        for (ftContactElem* contact = contacts[i]; contact != nullptr; contact = contact->next) {
            func(contact);
        }
    }
}

class ftContactBuffer2 {
private:

    struct ftContactElem {
        ftContact contact;
        uint32 index;
    };
    struct ftContactIndex {
        ftColHandle handleA, handleB;
        uint32 hashVal;
        ftContactElem* contactElem;
    };

    ftContactIndex* contactIndex;
    ftChunkArray<ftContactElem*> contacts;

    uint32 capacity;
    uint32 nContact;

    uint32 hash(ftColHandle keyA, ftColHandle keyB, uint32 capacity);
    uint64 pairingFunction(ftColHandle keyA, ftColHandle keyB);
    void rehash(uint32 newCapacity);
    void add(ftContactElem* elem, uint32 hashVal, uint32 handleA, uint32 handleB);

public:

    void init();
    void cleanup();

    class ftIter {
    public:
        ftColHandle handleA, handleB;
        ftContact* contact;
    private:
        uint32 index;
        friend class ftContactBuffer2;
    };

    ftContact* find(ftColHandle  key1, ftColHandle key2);
    ftContact* create(ftColHandle key1, ftColHandle key2);
    void destroy(ftContact* contact);

    uint32 getSize();

    ftIter iterate();
    void next(ftIter* iter);

    template <typename T>
    void forEach(T func);

};


#endif //FALTON_FTCONTACT_H
