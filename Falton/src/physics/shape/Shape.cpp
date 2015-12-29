//
// Created by Kevin Yu on 12/26/15.
//

#include "falton/physics/shape/Shape.h"

ftShapeTable::ftShapeTable() {

}

ftShapeTable::~ftShapeTable() {

}

ftShapeHandle ftShapeTable::createBox(const ftVector2& halfWidth) {
    ftShapeHandle shapeHandle;
    shapeHandle.id = new Box(halfWidth);
    return shapeHandle;
}

void ftShapeTable::destroyBox(ftShapeHandle shapeHandle) {
    delete shapeHandle.id;
}

real ftShapeTable::computeMass(ftShapeHandle shapeHandle, real density) {
    return shapeHandle.id->computeMass(density);
}
