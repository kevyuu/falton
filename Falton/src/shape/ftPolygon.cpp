//
// Created by Kevin Yu on 2/9/16.
//

#include <falton/setting.h>
#include "falton/physics/shape/ftPolygon.h"

ftPolygon::ftPolygon(): numVertex(0), vertices(nullptr), area(0),
                         normals(nullptr) {
    shapeType = SHAPE_POLYGON;
}

ftPolygon* ftPolygon::create(int numVertex, const ftVector2 *vertices){

    ftPolygon *polygon = new ftPolygon();

    ftVector2 *convexHull = new ftVector2[numVertex + 1];

    int numConvexVertex = GiftWrappingConvexHull(vertices, numVertex , convexHull);

    polygon->numVertex = numConvexVertex;
    polygon->vertices = new ftVector2[numConvexVertex];
    polygon->normals = new ftVector2[numConvexVertex];

    for (uint32 i=0;i<polygon->numVertex;i++) {
        polygon->vertices[i] = convexHull[i];
    }

    for (uint32 i=0;i<polygon->numVertex-1;i++) {
        polygon->normals[i] = (polygon->vertices[i + 1] - polygon->vertices[i]).perpendicular();
        polygon->normals[i].normalise();
    }
    polygon->normals[numVertex-1] = (polygon->vertices[0] - polygon->vertices[numVertex-1]).perpendicular();
    polygon->normals[numVertex-1].normalise();

    polygon->area = 0;
    for (uint32 i=0;i<polygon->numVertex-1;i++) {

        ftVector2 a = polygon->vertices[i];
        ftVector2 b = polygon->vertices[i+1];

        polygon->area += a.cross(b)/2;

    }
    polygon->area += ((polygon->vertices[numVertex-1]).cross(polygon->vertices[0])/2);

    delete[] convexHull;

    return polygon;
}

ftPolygon* ftPolygon::createBox(const ftVector2 &corner1, const ftVector2 &corner2) {

    ftPolygon *polygon = new ftPolygon;

    polygon->numVertex = 4;
    polygon->vertices = new ftVector2[4];
    polygon->normals = new ftVector2[4];

    real minX;
    real maxX;
    real minY;
    real maxY;

    if (corner1.x < corner2.x) {
        minX = corner1.x;
        maxX = corner2.x;
    } else {
        minX = corner2.x;
        maxX = corner1.x;
    }

    if (corner1.y < corner2.y) {
        minY = corner1.y;
        maxY = corner2.y;
    } else {
        minY = corner2.y;
        maxY = corner1.y;
    }

    ftVector2 *vertices = polygon->vertices;

    vertices[0] = ftVector2(minX, minY);
    vertices[1] = ftVector2(maxX, minY);
    vertices[2] = ftVector2(maxX, maxY);
    vertices[3] = ftVector2(minX, maxY);

    ftVector2 *normals = polygon->normals;
    normals[0] = ftVector2(0,-1);
    normals[1] = ftVector2(1,0);
    normals[2] = ftVector2(0,1);
    normals[3] = ftVector2(-1,0);

    polygon->area = (maxX - minX) * (maxY - minY);

    return polygon;
}

ftPolygon::~ftPolygon(){
    delete vertices;
    delete normals;
}

real ftPolygon::getArea() {
    return this->area;
}

void ftPolygon::copy(const ftShape* shape) {
    ftAssert(shape->shapeType == SHAPE_POLYGON, "");
    ftPolygon* sourcePolygon = (ftPolygon*) shape;
    numVertex = sourcePolygon->numVertex;
    area = sourcePolygon->area;

    if (vertices != nullptr) {
        delete[] vertices;
    }
    if (normals != nullptr) {
        delete[] normals;
    }

    vertices = new ftVector2[numVertex];
    normals = new ftVector2[numVertex];

    for (uint32 i = 0; i < numVertex; ++i) {
        vertices[i] = sourcePolygon->vertices[i];
    }
    for (uint32 i = 0; i < numVertex; ++i) {
        normals[i] = sourcePolygon->normals[i];
    }
}

ftAABB ftPolygon::constructAABB(ftTransform transform) const {

    ftAABB aabb;
    aabb.min = transform * vertices[0];
    aabb.max = aabb.min;

    for (uint32 i=1;i<numVertex;i++) {
        ftVector2 transformVertex = transform * vertices[i];
        if (aabb.min.x > transformVertex.x) aabb.min.x = transformVertex.x;
        if (aabb.min.y > transformVertex.y) aabb.min.y = transformVertex.y;
        if (aabb.max.x < transformVertex.x) aabb.max.x = transformVertex.x;
        if (aabb.max.y < transformVertex.y) aabb.max.y = transformVertex.y;
    }

    return aabb;
}

int GiftWrappingConvexHull(const ftVector2 *vertices, int numVertex, ftVector2 *convexHull) {

    convexHull[0] = vertices[0];

    for (int i=1;i<numVertex;i++) {
        if (convexHull[0].x > vertices[i].x ||
            (convexHull[0].x == vertices[i].x && convexHull[0].y > vertices[i].y)) {
            convexHull[0] = vertices[i];
        }
    }

    int numConvexVertex = 1;

    while(true) {

        convexHull[numConvexVertex] = vertices[0];

        for (int i=1;i<numVertex;i++) {
            ftVector2 line1 = convexHull[numConvexVertex] - convexHull[numConvexVertex - 1];
            ftVector2 line2 = vertices[i] - convexHull[numConvexVertex - 1];

            real orientation = line1.cross(line2);
            if (orientation < 0 ||(orientation == 0 &&
                                   line2.magnitude() > line1.magnitude())) {
                convexHull[numConvexVertex] = vertices[i];
            }
        }

        if (convexHull[numConvexVertex] == convexHull[0]) break;

        numConvexVertex++;

    }

    return numConvexVertex;

}