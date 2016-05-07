//
// Created by Kevin Yu on 4/15/16.
//

#ifndef FALTON_FTCONTACT_H
#define FALTON_FTCONTACT_H

#include <cstdint>
#include <falton/math/math.h>
#include <falton/physics/collision/ftCollisionSystem.h>

class ftShape;
struct ftContactEdge;
struct ftCollisionShape;

struct ftManifoldPoint {

    ftVector2 r1; //first shape contact point
    ftVector2 r2; //second shape contact point

    real nIAcc = 0.0;
    real tIAcc = 0.0;
};

struct ftManifold {
    ftVector2 normal; //normal pointing from second shape to first shape
    ftManifoldPoint contactPoints[2];
    real penetrationDepth[2];
    uint8 numContact = 0;
};

typedef enum ftCollisionState {
    BEGIN_COLLISION,
    IN_COLLISION
} ftCollisionState;

struct ftContact {

    ftColHandle handleA, handleB;

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

    ftContactBuffer();
    ~ftContactBuffer();

    class ftIter {
        private:
            uint32 index;
            ftContactElem* nextContact;
            friend class ftContactBuffer;
    };

    ftContact* find(ftColHandle  key1, ftColHandle key2);
    ftContact* create(ftColHandle key1, ftColHandle key2);
    void destroy(ftContact* contact);

    uint32 getSize();

    ftIter iterate();
    ftContact* start(ftIter* iter);
    ftContact* next(ftIter* iter);

};


#endif //FALTON_FTCONTACT_H
