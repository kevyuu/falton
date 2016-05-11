//
// Created by Kevin Yu on 4/6/16.
//

#include "gtest/gtest.h"
#include "falton/physics/shape/ftPolygon.h"
#include <iostream>

using namespace std;

void checkEqVertices(int vertexCount, ftVector2 *vertices1, ftVector2 *vertices2) {
    for (int i=0;i<vertexCount;++i) {
        EXPECT_FLOAT_EQ(vertices1[i].x, vertices2[i].x);
        EXPECT_FLOAT_EQ(vertices1[i].y, vertices2[i].y);
    }
}

TEST(TEST_CONVEX_HULL, TEST_CASE_1) {
    ftVector2 vertices[4] = {ftVector2(0,0) , ftVector2(0,1) , ftVector2(1,0) , ftVector2(1,1)};
    ftVector2 expectedVertices[4] = {ftVector2(0,0), ftVector2(1, 0), ftVector2(1,1) , ftVector2(0,1)};
    ftVector2 expectedNormals[4] = {ftVector2(0,-1), ftVector2(1,0), ftVector2(0,1), ftVector2(-1,0)};
    ftPolygon polygon;

    polygon.init(4, vertices);

    EXPECT_EQ(4, polygon.numVertex);

    checkEqVertices(4, expectedVertices, polygon.vertices);
    checkEqVertices(4, expectedNormals, polygon.normals);

    polygon.destroy();

}

TEST(TEST_CONVEX_HULL, TEST_CASE_2) {
    ftVector2 vertices[5] = {ftVector2(0,0), ftVector2(0, 0.5), ftVector2(0,1), ftVector2(1,0), ftVector2(1,1)};
    ftVector2 expectedVertices[4] = {ftVector2(0,0), ftVector2(1, 0), ftVector2(1,1) , ftVector2(0,1)};
    ftVector2 expectedNormals[4] = {ftVector2(0,-1), ftVector2(1,0), ftVector2(0,1), ftVector2(-1,0)};
    ftPolygon polygon;

    polygon.init(5,vertices);

    EXPECT_EQ(4, polygon.numVertex);

    checkEqVertices(4, expectedVertices, polygon.vertices);
    checkEqVertices(4, expectedNormals, polygon.normals);

    polygon.destroy();

}

TEST(TEST_CONVEX_HULL, TEST_CASE_3) {

    ftVector2 vertices[5] = {ftVector2(0,0), ftVector2(0.5, 0.5), ftVector2(0,1), ftVector2(1,0), ftVector2(1,1)};
    ftVector2 expectedVertices[4] = {ftVector2(0,0), ftVector2(1, 0), ftVector2(1,1) , ftVector2(0,1)};
    ftVector2 expectedNormals[4] = {ftVector2(0,-1), ftVector2(1,0), ftVector2(0,1), ftVector2(-1,0)};
    ftPolygon polygon;

    polygon.init(5,vertices);

    EXPECT_EQ(4, polygon.numVertex);

    checkEqVertices(4, expectedVertices, polygon.vertices);
    checkEqVertices(4, expectedNormals, polygon.normals);

    polygon.destroy();

}

TEST(TEST_BOX, TEST_CASE_1) {
    ftVector2 expectedVertices[4] = {ftVector2(0,0), ftVector2(1, 0), ftVector2(1,1) , ftVector2(0,1)};
    ftVector2 expectedNormals[4] = {ftVector2(0,-1), ftVector2(1,0), ftVector2(0,1), ftVector2(-1,0)};

    ftPolygon polygon;
    ftVector2 corner1(1,0);
    ftVector2 corner2(0,1);

    polygon.initAsBox(&corner1, &corner2);

    EXPECT_EQ(4, polygon.numVertex);

    checkEqVertices(4, expectedVertices, polygon.vertices);
    checkEqVertices(4, expectedNormals, polygon.normals);

    polygon.destroy();

}

TEST(TEST_MOMENT_INERTIA, TEST_CASE_1) {

    ftPolygon polygon;
    ftVector2 corner1(-2,-2);
    ftVector2 corner2(2,2);

    polygon.initAsBox(&corner1, &corner2);

    EXPECT_FLOAT_EQ(64.0/12, polygon.computeMomentInertia(2.0/16));

    polygon.destroy();
}


