//
// Created by Kevin Yu on 4/7/16.
//

#include "gtest/gtest.h"
#include "falton/physics/Collision/ftManifoldComputer.h"
#include <iostream>

using namespace std;

TEST(TEST_BOX_BOX, TEST_CASE_1) {

    ftPolygon box1;
    ftVector2 box1corner1 = ftVector2(4,2);
    ftVector2 box1corner2 = ftVector2(12,5);
    box1.initAsBox(&box1corner1, &box1corner2);

    ftPolygon box2;
    ftVector2 box2corner1 = ftVector2(8,4);
    ftVector2 box2corner2 = ftVector2(14, 9);
    box2.initAsBox(&box2corner1, &box2corner2);

    ftTransformShape transformShape1;
    transformShape1.shape = &box1;

    ftTransformShape transformShape2;
    transformShape2.shape = &box2;

    ftManifold manifold;
    ftCollision::Collide(transformShape1, transformShape2, manifold);

    cout<<"Num Contact : "<<(int)manifold.numContact<<endl;


    cout<<"Normal : "<<manifold.normal.x<<" "<<manifold.normal.y<<endl;

    for (int i=0;i<manifold.numContact;i++) {

        cout<<"Penetration Depth : "<<manifold.penetrationDepth[i]<<endl;
        cout<<"Point r1 : "<<manifold.contactPoints[i].r1.x<<" "<<manifold.contactPoints[i].r1.y<<endl;
        cout<<"Point r2 : "<<manifold.contactPoints[i].r2.x<<" "<<manifold.contactPoints[i].r2.y<<endl;
        cout<<"--------"<<endl;
    }

}

TEST (TEST_BOX_BOX, TEST_CASE_2) {
    ftPolygon box1;
    ftVector2 box1corner1 = ftVector2(4,2);
    ftVector2 box1corner2 = ftVector2(12,5);
    box1.initAsBox(&box1corner1, &box1corner2);

    ftPolygon box2;
    ftVector2 vertices[4] = {ftVector2(2,8), ftVector2(6,4), ftVector2(9,7), ftVector2(5,11)};
    box2.init(4, vertices);

    ftTransformShape transformShape1;
    transformShape1.shape = &box1;

    ftTransformShape transformShape2;
    transformShape2.shape = &box2;

    ftManifold manifold;
    ftCollision::Collide(transformShape1, transformShape2, manifold);

    cout<<"Num Contact : "<<(int)manifold.numContact<<endl;


    cout<<"Normal : "<<manifold.normal.x<<" "<<manifold.normal.y<<endl;

    for (int i=0;i<manifold.numContact;i++) {

        cout<<"Penetration Depth : "<<manifold.penetrationDepth[i]<<endl;
        cout<<"Point r1 : "<<manifold.contactPoints[i].r1.x<<" "<<manifold.contactPoints[i].r1.y<<endl;
        cout<<"Point r2 : "<<manifold.contactPoints[i].r2.x<<" "<<manifold.contactPoints[i].r2.y<<endl;
        cout<<"--------"<<endl;
    }
}

TEST (TEST_BOX_BOX, TEST_CASE_3) {
    ftPolygon box1;
    ftVector2 box1corner1 = ftVector2(4,2);
    ftVector2 box1corner2 = ftVector2(12,5);
    box1.initAsBox(&box1corner1, &box1corner2);

    ftPolygon box2;
    ftVector2 vertices[4] = {ftVector2(9,4), ftVector2(13,3), ftVector2(14,7), ftVector2(10,8)};
    box2.init(4, vertices);

    ftTransformShape transformShape1;
    transformShape1.shape = &box1;

    ftTransformShape transformShape2;
    transformShape2.shape = &box2;

    ftManifold manifold;
    ftCollision::Collide(transformShape1, transformShape2, manifold);

    cout<<"Num Contact : "<<(int)manifold.numContact<<endl;


    cout<<"Normal : "<<manifold.normal.x<<" "<<manifold.normal.y<<endl;

    for (int i=0;i<manifold.numContact;i++) {

        cout<<"Penetration Depth : "<<manifold.penetrationDepth[i]<<endl;
        cout<<"Point r1 : "<<manifold.contactPoints[i].r1.x<<" "<<manifold.contactPoints[i].r1.y<<endl;
        cout<<"Point r2 : "<<manifold.contactPoints[i].r2.x<<" "<<manifold.contactPoints[i].r2.y<<endl;
        cout<<"--------"<<endl;
    }
}

TEST (TEST_BOX_BOX, TEST_CASE_4) {
    ftPolygon box1;
    ftVector2 box1corner1 = ftVector2(0,0);
    ftVector2 box1corner2 = ftVector2(1,1);
    box1.initAsBox(&box1corner1, &box1corner2);

    ftPolygon box2;
    ftVector2 box2corner1 = ftVector2(1,1);
    ftVector2 box2corner2 = ftVector2(4, 4);
    box2.initAsBox(&box2corner1, &box2corner2);

    ftTransformShape transformShape1;
    transformShape1.shape = &box1;

    ftTransformShape transformShape2;
    transformShape2.shape = &box2;

    ftManifold manifold;
    ftCollision::Collide(transformShape1, transformShape2, manifold);

    cout<<"Num Contact : "<<(int)manifold.numContact<<endl;


    cout<<"Normal : "<<manifold.normal.x<<" "<<manifold.normal.y<<endl;

    for (int i=0;i<manifold.numContact;i++) {

        cout<<"Penetration Depth : "<<manifold.penetrationDepth[i]<<endl;
        cout<<"Point r1 : "<<manifold.contactPoints[i].r1.x<<" "<<manifold.contactPoints[i].r1.y<<endl;
        cout<<"Point r2 : "<<manifold.contactPoints[i].r2.x<<" "<<manifold.contactPoints[i].r2.y<<endl;
        cout<<"--------"<<endl;
    }
}


TEST (TEST_BOX_BOX, TEST_CASE_5) {
    ftPolygon box1;
    ftVector2 box1corner1 = ftVector2(0,0);
    ftVector2 box1corner2 = ftVector2(640,120);
    box1.initAsBox(&box1corner1, &box1corner2);

    ftPolygon box2;
    ftVector2 box2corner1 = ftVector2(316,119.885);
    ftVector2 box2corner2 = ftVector2(324, 127.885);
    box2.initAsBox(&box2corner1, &box2corner2);

    ftTransformShape transformShape1;
    transformShape1.shape = &box1;

    ftTransformShape transformShape2;
    transformShape2.shape = &box2;

    ftManifold manifold;
    ftCollision::Collide(transformShape1, transformShape2, manifold);

    cout<<"Num Contact : "<<(int)manifold.numContact<<endl;


    cout<<"Normal : "<<manifold.normal.x<<" "<<manifold.normal.y<<endl;

    for (int i=0;i<manifold.numContact;i++) {

        cout<<"Penetration Depth : "<<manifold.penetrationDepth[i]<<endl;
        cout<<"Point r1 : "<<manifold.contactPoints[i].r1.x<<" "<<manifold.contactPoints[i].r1.y<<endl;
        cout<<"Point r2 : "<<manifold.contactPoints[i].r2.x<<" "<<manifold.contactPoints[i].r2.y<<endl;
        cout<<"--------"<<endl;
    }
}


TEST (TEST_BOX_BOX, TEST_CASE_6) {
    ftPolygon box1;
    ftVector2 box1corner1 = ftVector2(-50,-10);
    ftVector2 box1corner2 = ftVector2(50,10);
    box1.initAsBox(&box1corner1, &box1corner2);

    ftPolygon box2;
    ftVector2 box2corner1 = ftVector2(-0.5,0.99617);
    ftVector2 box2corner2 = ftVector2(0.5,1.99167);
    box2.initAsBox(&box2corner1, &box2corner2);

    ftTransformShape transformShape1;
    transformShape1.transform.rotation = ftRotation(PI / 8);
    transformShape1.transform.center = ftVector2(0,-10);
    transformShape1.shape = &box1;

    ftTransformShape transformShape2;
    transformShape2.shape = &box2;

    ftManifold manifold;
    ftCollision::Collide(transformShape1, transformShape2, manifold);

    cout<<"Num Contact : "<<(int)manifold.numContact<<endl;


    cout<<"Normal : "<<manifold.normal.x<<" "<<manifold.normal.y<<endl;

    for (int i=0;i<manifold.numContact;i++) {

        cout<<"Penetration Depth : "<<manifold.penetrationDepth[i]<<endl;
        cout<<"Point r1 : "<<manifold.contactPoints[i].r1.x<<" "<<manifold.contactPoints[i].r1.y<<endl;
        cout<<"Point r2 : "<<manifold.contactPoints[i].r2.x<<" "<<manifold.contactPoints[i].r2.y<<endl;
        cout<<"--------"<<endl;
    }
}
