//
// Created by Kevin Yu on 12/12/15.
//

#ifndef FALTON_COLLIDER_H
#define FALTON_COLLIDER_H

#include "falton/math/precision.h"
#include "ftHandle.h"
#include "ftDef.h"

struct ftCollider {
    ftVector2 position;
    real orientation;

    real restitution;
    real friction;

    ftShapeHandle shape;
    ftColliderHandle next;
};

class ftColliderTable {

public:

    ftColliderTable();
    ~ftColliderTable();

    ftColliderHandle create(const ftColliderDef& colliderDef);
    void destroy(ftColliderHandle colHandle);

    ftColliderHandle addCollider(ftBodyHandle bodyHandle, const ftColliderDef& colliderDef);

    void setRestitution(ftColliderHandle colHandle, real newRestitution);
    void setFriction(ftColliderHandle colHandle, real newFriction);
    void setShape(ftColliderHandle colHandle, ftShapeHandle shapeHandle);

    real getRestitution(ftColliderHandle colHandle);
    real getFriction(ftColliderHandle colHandle);
    ftColliderHandle getColliders(ftBodyHandle bodyHandle);
    ftShapeHandle getShape(ftColliderHandle colHandle);

    ftColliderHandle next(ftColliderHandle colliderHandle);

private:

    ftColliderHandle bodyToColliderMap[100];

};


#endif //FALTON_COLLIDER_H
