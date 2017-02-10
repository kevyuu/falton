//
// Created by Kevin Yu on 12/12/15.
//

#ifndef FALTON_SHAPE_H
#define FALTON_SHAPE_H

#include <falton/container/ftChunkArray.h>
#include <falton/math.h>
#include <falton/shape/ftAABB.h>

struct ftPolygon;
struct ftCircle;

enum ShapeType {
    SHAPE_CIRCLE,
    SHAPE_POLYGON,
    SHAPE_TYPE_NUMBER_ITEM
};

struct ftShape {

public:

    ShapeType shapeType;

    virtual real getArea() = 0;
    virtual ftAABB constructAABB(ftTransform transform) const = 0;
    virtual void copy(const ftShape* shape) = 0;

    virtual ~ftShape() {};

};

#endif //FALTON_SHAPE_H
