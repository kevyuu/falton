//
// Created by Kevin Yu on 4/15/16.
//

#ifndef FALTON_FTCONTACT_H
#define FALTON_FTCONTACT_H

#include <falton/math/math.h>

typedef uint32 ColHandle;
class ftShape;
struct ftContactEdge;
struct ftTransformShape;

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

    ColHandle handleA, handleB; //handle A alwalys < handle B
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
        ftContactElem *next = nullptr;
        ftContactElem *prev = nullptr;
    };

    ftContactElem **contacts;
    uint32 capacity;
    uint32 nContact;

    uint32 hash(ColHandle handleA, ColHandle handleB, uint32 capacity);
    uint64 pairingFunction(ColHandle handleA, ColHandle handleB);
    void rehash(uint32 newCapacity);

public:

    ftContactBuffer();
    ~ftContactBuffer();

    class ftIter {
        private:
            uint32 index;
            ftContactElem* nextContact;
            friend class ftContactBuffer;
    };

    ftContact* create(ColHandle handle1, ColHandle handle2);
    void destroy(ftContact* contact);

    uint32 getSize();

    ftIter iterate();
    ftContact* start(ftIter* iter);
    ftContact* next(ftIter* iter);

    ftContact* find(ColHandle handle1, ColHandle handle2);
};


#endif //FALTON_FTCONTACT_H
