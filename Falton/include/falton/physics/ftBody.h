//
// Created by Kevin Yu on 12/6/15.
//

#ifndef FALTON_RIGIDBODY_H
#define FALTON_RIGIDBODY_H

#include <deque>
#include "falton/math/math.h"
#include "ftHandle.h"
#include "falton/container/ChunkArray.h"
#include "ftDef.h"

struct ftColliderHandle;




struct ftBody {

    ftBodyHandle bodyHandle;

    ftVector2 position;
    ftVector2 velocity;

    real orientation;
    real angular_velocity;

    real mass;
    real inverse_mass;
    real rotation_inerita;
    real inverse_rotation_inertia;

};

class ftBodyTable {

public:
    ftBodyTable(int initialSize);

    ~ftBodyTable();

    ftBodyHandle create(const ftBodyDef& bodyDef);

    void destroy(ftBodyHandle bodyHandle);

    void setPosition(ftBodyHandle bodyHandle, const ftVector2& newPosition);

    void setVelocity(ftBodyHandle bodyHandle, const ftVector2& newVelocity);

    void setOrientation(ftBodyHandle bodyHandle, const real newOrientation);

    void setAngularVelocity(ftBodyHandle bodyHandle, const real newAngularVelocity);

    void setMass(ftBodyHandle bodyHandle, const real newMass);

    void setRotationInertia(ftBodyHandle bodyHandle, const real newInertia);

    ftVector2 getPosition(ftBodyHandle bodyHandle);

    ftVector2 getVelocity(ftBodyHandle bodyHandle);

    real getOrientation(ftBodyHandle bodyHandle);

    real getAngularVelocity(ftBodyHandle bodyHandle);

    real getMass(ftBodyHandle bodyHandle);

    real getInverseMass(ftBodyHandle bodyHandle);

    real getRotationInertia(ftBodyHandle bodyHandle);

    real getInverseRotationInertia(ftBodyHandle bodyHandle);

private:
    ftBody** handleMap;
    int lastId;

    ChunkArray<ftBody> bodies;
    int nBody;

    std::deque<int> freeID;

    int nextID();
};

#endif //FALTON_RIGIDBODY_H
