//
// Created by Kevin Yu on 12/12/15.
//

#include <falton/setting/general.h>
#include <falton/physics/shape/ftCircle.h>

ftCircle::ftCircle() {
    shapeType = SHAPE_CIRCLE;
}

ftCircle* ftCircle::create(real radius) {
    ftCircle* circle = new ftCircle();
    circle->radius = radius;
    circle->area = PI * radius * radius;
    return circle;
}

real ftCircle::getArea() {
    return area;
}

void ftCircle::copy(const ftShape* shape) {
    ftAssert(shape->shapeType == SHAPE_CIRCLE);
    ftCircle* sourceCircle = (ftCircle*) shape;
    radius = sourceCircle->radius;
    area = sourceCircle->radius;
}

ftAABB ftCircle::constructAABB(ftTransform transform) {
    ftAABB aabb;
    ftVector2 halfWidthVector(radius,radius);
    aabb.min = transform.center - halfWidthVector;
    aabb.max = transform.center + halfWidthVector;
    return aabb;
}