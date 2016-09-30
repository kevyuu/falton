//
// Created by Kevin Yu on 2/9/16.
//

#ifndef FALTON_POLYGON_H
#define FALTON_POLYGON_H

#include <falton/math.h>
#include <falton/shape/ftShape.h>

class ftShape;

class ftPolygon : public ftShape {
public:
    ftVector2* vertices; // stored in counter clockwise order
    ftVector2* normals; // stored in counter clockwise order

    uint32 numVertex;

    real area;

    static ftPolygon* create(int numVertex, const ftVector2* vertices);
    static ftPolygon* createBox(const ftVector2& corner1, const ftVector2& corner2);

    real getArea();
    void copy(const ftShape* shape);
    ftAABB constructAABB(ftTransform transform) const override;

    ftPolygon();
    ~ftPolygon();

};

int GiftWrappingConvexHull(const ftVector2* vertices, int numVertex, ftVector2 *convexHull);

#endif //FALTON_POLYGON_H
