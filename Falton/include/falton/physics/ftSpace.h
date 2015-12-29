//
// Created by Kevin Yu on 12/26/15.
//

#ifndef FALTON_FTSPACE_H
#define FALTON_FTSPACE_H

#include "ftHandle.h"

class ftSpace {
public:

    ftBodyHandle createBody(const ftBodyDef& bodyDef) {
        bodyTable.create(bodyDef);
    }

    ftColliderHandle addCollider(ftBodyHandle bodyHandle,
                                 const ftColliderDef& colliderDef, const Box& box) {
        colliderTable.addCollider(bodyHandle, colliderDef);
        shapeTable.createBox(box.half_width);
    }


private:
    ftBodyTable bodyTable;
    ftColliderTable colliderTable;
    ftShapeTable shapeTable;

};


#endif //FALTON_FTSPACE_H
