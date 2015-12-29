//
// Created by Kevin Yu on 12/12/15.
//

#ifndef FALTON_SHAPE_H
#define FALTON_SHAPE_H

#include "Box.h"
#include "Circle.h"
#include "../ftHandle.h"
#include "falton/container/ChunkArray.h"

class ftShapeTable {
public:

    ftShapeTable();
    ~ftShapeTable();

    ftShapeHandle createBox(const ftVector2& halfWidth);
    void destroyBox(ftShapeHandle shapeHandle);

    real computeMass(ftShapeHandle shapeHandle, real  density);

private:

};

#endif //FALTON_SHAPE_H
