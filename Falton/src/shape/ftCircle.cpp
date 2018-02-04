//
// Created by Kevin Yu on 12/12/15.
//

#include <falton/setting.h>
#include <falton/shape/ftCircle.h>

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
    ftAssert(shape->shapeType == SHAPE_CIRCLE, "");
    ftCircle* sourceCircle = (ftCircle*) shape;
    radius = sourceCircle->radius;
    area = 3.14 * radius * radius;
}

ftAABB ftCircle::constructAABB(ftTransform transform) const {
    ftAABB aabb;
    ftVector2 halfWidthVector(radius,radius);
    aabb.min = transform.center - halfWidthVector;
    aabb.max = transform.center + halfWidthVector;
    return aabb;
}

void ftCircle::setRadius(real radius) {
    this->radius = radius;
    this->area = 3.14 * radius * radius;
}