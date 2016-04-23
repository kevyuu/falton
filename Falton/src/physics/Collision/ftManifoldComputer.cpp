//
// Created by Kevin Yu on 2/3/16.
//

#include "falton/physics/Collision/ftManifoldComputer.h"
#include "falton/physics/shape/ftPolygon.h"
#include "falton/physics/collision/ftContact.h"

const CollisionFunc ftManifoldComputer::collisionFunctions[SHAPE_TYPE_NUMBER_ITEM][SHAPE_TYPE_NUMBER_ITEM] = {

        ftManifoldComputer::CircleToCircleCollission,
        nullptr,
        nullptr,
        ftManifoldComputer::PolygonToPolgonCollision
};

void ftManifoldComputer::Collide(const ftTransformShape &shapeA,
                    const ftTransformShape &shapeB,
                    ftManifold& manifold) {
    collisionFunctions[shapeA.shape->shapeType][shapeB.shape->shapeType](shapeA,shapeB,manifold);
}

void ftManifoldComputer::PolygonToPolgonCollision(const ftTransformShape& shapeA,
                                     const ftTransformShape& shapeB,
                                     ftManifold& manifold) {

    ftPolygon* polygonA = (ftPolygon *) shapeA.shape;
    ftPolygon* polygonB = (ftPolygon *) shapeB.shape;

    ftVector2* worldNormalsA = new ftVector2[polygonA->numVertex];
    ftVector2* worldNormalsB = new ftVector2[polygonB->numVertex];

    ftVector2* worldVertexesA = new ftVector2[polygonA->numVertex];
    ftVector2* worldVertexesB = new ftVector2[polygonB->numVertex];

    for (int i=0;i<polygonA->numVertex;i++) {
        worldNormalsA[i] = shapeA.transform.rotation * polygonA->normals[i];
        worldVertexesA[i] = shapeA.transform * polygonA->vertices[i];
    }

    for (int i=0;i<polygonB->numVertex;i++) {
        worldNormalsB[i] = shapeB.transform.rotation * polygonB->normals[i];
        worldVertexesB[i] = shapeB.transform * polygonB->vertices[i];
    }

    MTVInput mtvInput;
    mtvInput.numVertexA = polygonA->numVertex;
    mtvInput.normalsA = worldNormalsA;
    mtvInput.vertexesA = worldVertexesA;
    mtvInput.numVertexB = polygonB->numVertex;
    mtvInput.normalsB = worldNormalsB;
    mtvInput.vertexesB = worldVertexesB;

    //Find Minimum Translation Vector
    MTVOutput mtvOutput = FindPolygonToPolygonMTV(mtvInput);

    if (mtvOutput.separation >= 0) return;

    ftPolygon *refPolygon;
    ftPolygon *incPolygon;
    ftVector2 separatingAxis;
    ftVector2 *refVertexes;
    ftVector2 *refNormals;
    ftVector2 *incVertexes;
    ftVector2 *incNormals;
    int incNumVertex;

    if (mtvOutput.polygon == MTVOutput::polygon_A) {
        refPolygon = polygonA;
        incPolygon = polygonB;
        separatingAxis = worldNormalsA[mtvOutput.normalIndex];
        refVertexes = worldVertexesA;
        refNormals = worldNormalsA;
        incVertexes = worldVertexesB;
        incNormals = worldNormalsB;
        incNumVertex = polygonB->numVertex;
    } else {
        refPolygon = polygonB;
        incPolygon = polygonA;
        separatingAxis = worldNormalsB[mtvOutput.normalIndex];
        refVertexes = worldVertexesB;
        refNormals = worldNormalsB;
        incVertexes = worldVertexesA;
        incNormals = worldNormalsA;
        incNumVertex = polygonA->numVertex;
    }

    int incidentEdgeIndex = FindIncidentEdge(separatingAxis, incNormals, incNumVertex);

    int refVertex1 = mtvOutput.normalIndex;
    int refVertex2 = mtvOutput.normalIndex + 1 == refPolygon->numVertex ? 0 : mtvOutput.normalIndex + 1;

    int incVertex1 = incidentEdgeIndex;
    int incVertex2 = incidentEdgeIndex + 1 == incPolygon->numVertex ? 0 : incidentEdgeIndex + 1;

    ClipPoint clipPoint1 = ClipIncidentToReferenceLine(refVertexes[refVertex2] - refVertexes[refVertex1],
                                                       refVertexes[refVertex1], incVertexes[incVertex1],
                                                       incVertexes[incVertex2]);

    ClipPoint clipPoint2 = ClipIncidentToReferenceLine(refVertexes[refVertex1] - refVertexes[refVertex2],
                                                       refVertexes[refVertex2], clipPoint1.point[0],
                                                       clipPoint1.point[1]);

    manifold.normal = refNormals[mtvOutput.normalIndex];
    uint8 contactPointCount = 0;
    for (int i=0;i<clipPoint2.numPoint;i++) {
        real separation = separatingAxis.dot(clipPoint2.point[i] - refVertexes[refVertex1]);
        if (separation < 0) {
            manifold.penetrationDepth[contactPointCount] = -1 * separation;
            manifold.contactPoints[contactPointCount].r1 = clipPoint2.point[i] - (separation * separatingAxis);
            manifold.contactPoints[contactPointCount].r2 = clipPoint2.point[i];
            contactPointCount++;
        }
    }

    manifold.numContact = contactPointCount;

    if (mtvOutput.polygon == mtvOutput.polygon_B) {
        manifold.normal *= -1;
        for (int i = 0; i < manifold.numContact; ++i) {
            ftVector2 tmp = manifold.contactPoints[i].r1;
            manifold.contactPoints[i].r1 = manifold.contactPoints[i].r2;
            manifold.contactPoints[i].r2 = tmp;
        }
    }
}

void ftManifoldComputer::CircleToCircleCollission(const ftTransformShape& shapeA,
                                     const ftTransformShape& shapeB,
                                     ftManifold& manifold) {


    ftCircle *circleA = (ftCircle *) shapeA.shape;
    ftCircle *circleB = (ftCircle *) shapeB.shape;

    real totalRadius = circleA->radius + circleB->radius;
    ftVector2 separatingAxis = shapeB.transform.center - shapeA.transform.center;
    real distance = separatingAxis.magnitude();
    if (totalRadius > distance) {
        return;
    }

    real penetrationDepth = totalRadius - distance;
    manifold.normal = separatingAxis.unit();

    manifold.contactPoints[0].r1 = shapeA.transform.center + circleA->radius * manifold.normal;
    manifold.contactPoints[0].r2 = shapeB.transform.center - circleB->radius * manifold.normal;
    manifold.penetrationDepth[0] = penetrationDepth;
    manifold.numContact = 1;

}

ftManifoldComputer::MTVOutput ftManifoldComputer::FindPolygonToPolygonMTV(const MTVInput& mtvInput) {

    //search normal with maximum minimum separation

    //test normals of polygonA.
    real maxSeparationA = real_minInfinity;
    real maxNormalA = 0;
    for (int i=0;i<mtvInput.numVertexA;i++) {

        real minSeparation = real_Infinity; // minimum separation for normal i
        for (int j=0;j<mtvInput.numVertexB;j++) {

            real separation = (mtvInput.vertexesB[j] - mtvInput.vertexesA[i]).dot(mtvInput.normalsA[i]);

            if (minSeparation > separation) {
                minSeparation = separation;
            }
        }

        if (maxSeparationA < minSeparation) {
            maxSeparationA = minSeparation;
            maxNormalA = i;
        }
    }

    //test normals of polygonB
    real maxSeparationB = real_minInfinity;
    real maxNormalB = 0;
    for (int i=0;i<mtvInput.numVertexB;i++) {

        real minSeparation = real_Infinity;
        for (int j=0;j<mtvInput.numVertexA;j++) {
            real separation = (mtvInput.vertexesA[j] - mtvInput.vertexesB[i]).dot(mtvInput.normalsB[i]);

            if (minSeparation > separation) {
                minSeparation = separation;
            }

        }

        if (maxSeparationB < minSeparation) {
            maxSeparationB = minSeparation;
            maxNormalB = i;
        }

    }

    MTVOutput mtvOutput;

    if (maxSeparationA >= maxSeparationB) {
        mtvOutput.polygon = MTVOutput::polygon_A;
        mtvOutput.normalIndex = maxNormalA;
        mtvOutput.separation = maxSeparationA;
    } else {
        mtvOutput.polygon = MTVOutput::polygon_B;
        mtvOutput.normalIndex = maxNormalB;
        mtvOutput.separation = maxSeparationB;
    }

    return mtvOutput;

}

ftManifoldComputer::ClipPoint ftManifoldComputer::ClipIncidentToReferenceLine(const ftVector2& refAxis, ftVector2 clipBoundary ,
                                             const ftVector2& incVertex1, const ftVector2& incVertex2) {


    ClipPoint clipPoint;
    clipPoint.numPoint = 0;

    ftVector2 normAxis = refAxis.unit();

    real offset = normAxis.dot(clipBoundary);

    real offsetIncVertex1 = normAxis.dot(incVertex1) - offset;
    real offsetIncVertex2 = normAxis.dot(incVertex2) - offset;

    if (offsetIncVertex1 > 0) {
        clipPoint.point[clipPoint.numPoint] = incVertex1;
        clipPoint.numPoint++;
    }
    if (offsetIncVertex2 > 0) {
        clipPoint.point[clipPoint.numPoint] = incVertex2;
        clipPoint.numPoint++;
    }

    if (offsetIncVertex1 * offsetIncVertex2 < 0) {

        real clipOffset = offsetIncVertex1 / (offsetIncVertex1 - offsetIncVertex2);

        ftVector2 incDirection = (incVertex2 - incVertex1);

        clipPoint.point[clipPoint.numPoint] = incVertex1 + (incDirection * clipOffset);
        clipPoint.numPoint++;

    }

    return clipPoint;

}

int ftManifoldComputer::FindIncidentEdge(const ftVector2& separatingAxis, const ftVector2* incidentNormals, int normalsCount) {
    real minPerpendicularDegree = real_Infinity
    real incidentIndex = 0;
    for (int i=0;i<normalsCount;i++) {
        real perpendicularDegree = separatingAxis.dot(incidentNormals[i]);
        if (perpendicularDegree < minPerpendicularDegree) {
            minPerpendicularDegree = perpendicularDegree;
            incidentIndex = i;
        }
    }

    return incidentIndex;

}
