//
// Created by Kevin Yu on 2/3/16.
//

#ifndef FALTON_COLLISION_H
#define FALTON_COLLISION_H

#include <falton/math.h>
#include <falton/shape/ftShape.h>

struct ftManifold;
struct ftCollisionShape;

typedef void (* ftCollisionFunc)(const ftCollisionShape&, const ftCollisionShape&, ftManifold*);

class ftManifoldComputer {

public :

    static void Collide(const ftCollisionShape &shapeA,
                        const ftCollisionShape &shapeB,
                        ftManifold *manifold);
private:

    struct MTVInput {
        uint32 numVertexA;
        ftVector2* normalsA;
        ftVector2* vertexesA;

        uint32 numVertexB;
        ftVector2* normalsB;
        ftVector2* vertexesB;
    };

    struct MTVOutput {
        enum PolygonID
        {
            polygon_A,
            polygon_B
        };

        PolygonID polygon;
        uint32 normalIndex;
        real separation;
    };

    struct ClipPoint {
        ftVector2 point[2];
        uint32 numPoint;
    };

    static const ftCollisionFunc collisionFunctions[SHAPE_TYPE_NUMBER_ITEM][SHAPE_TYPE_NUMBER_ITEM];

    static void PolygonToPolgonCollision(const ftCollisionShape &shapeA,
                                         const ftCollisionShape &shapeB,
                                         ftManifold *manifold);

    static void PolygonToCircleCollision(const ftCollisionShape &shapeA,
                                         const ftCollisionShape &shapeB,
                                         ftManifold *manifold);

    static void CircleToPolygonCollision(const ftCollisionShape &shapeA,
                                         const ftCollisionShape &shapeB,
                                         ftManifold *manifold);

    static void CircleToCircleCollission(const ftCollisionShape& shapeA,
                                         const ftCollisionShape& shapeB,
                                         ftManifold *manifold);

    static MTVOutput FindPolygonToPolygonMTV(const MTVInput& mtvInput);

    static ClipPoint ClipIncidentToReferenceLine(const ftVector2& refAxis, 
                                                 ftVector2 clipBoundary ,
                                                 const ftVector2& incVertex1, 
                                                 const ftVector2& incVertex2);

    static uint32 FindIncidentEdge(const ftVector2& separatingAxis,
                                   const ftVector2* incidentNormals,
                                   int normalsCount);

};


#endif //FALTON_COLLISION_H
