//
// Created by Kevin Yu on 12/12/15.
//

#include "falton/physics/shape/ftCircle.h"

ftCircle::ftCircle(real radius) : radius(radius) {

    shapeType = SHAPE_CIRCLE;
    area = PI * radius * radius;
}

real ftCircle::getArea() {
    return area;
}

real ftCircle::computeMomentInertia(real density) {
    return density * area * radius * radius / 2;
}

ftAABB ftCircle::constructAABB(ftTransform transform) {
    ftAABB aabb;
    ftVector2 halfWidthVector(radius,radius);
    aabb.min = transform.center - halfWidthVector;
    aabb.max = transform.center + halfWidthVector;
    return aabb;
}