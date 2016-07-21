//
// Created by Kevin Yu on 2016-05-19.
//

#ifndef FALTON_DEMO_H
#define FALTON_DEMO_H

#include <ctime>
#include <cstdlib>

#include <falton/physics/dynamic/ftBody.h>
#include <falton/physics/shape/ftPolygon.h>
#include <falton/physics/ftMassComputer.h>
#include <falton/physics/dynamic/ftCollider.h>
#include <falton/physics/dynamic/ftPhysicsSystem.h>
#include "DrawPhysics.h"

extern ftPhysicsSystem physicsSystem;
extern Camera camera;

void initPhysics() {
    physicsSystem.init();
}

void updatePhysics(real dt) {
    physicsSystem.step(dt);
}

void cleanupPhysics() {
    physicsSystem.shutdown();
}

ftBody* createDynamicBox(ftVector2 position, ftVector2 halfWidth, real mass, real friction) {
    ftPolygon *boxShape = ftPolygon::createBox(-1 * halfWidth, halfWidth);
    ftMassProperty boxmp = ftMassComputer::computeForPolygon(*boxShape,mass, ftVector2(0,0));
    ftBody* box = physicsSystem.createDynamicBody(position,0, boxmp.mass, boxmp.moment);
    box->centerOfMass = boxmp.centerOfMass;
    box->moment = boxmp.moment;

    ftCollider* boxCollider = physicsSystem.createCollider(box, boxShape, ftVector2(0,0), 0);
    boxCollider->friction = friction;
    boxCollider->restitution = 0;
    delete boxShape;

    return box;
}


ftBody* createBall(ftVector2 position, real mass, real radius, real friction, real restitution) {
    ftCircle *ballShape = ftCircle::create(radius);
    ftMassProperty ballmp = ftMassComputer::computeForCircle(*ballShape,mass, ftVector2(0,0));
    ftBody* ball = physicsSystem.createDynamicBody(position, 0, ballmp.mass, ballmp.moment);
    ball->centerOfMass = ballmp.centerOfMass;

    ftCollider* ballCollider = physicsSystem.createCollider(ball, ballShape, ftVector2(0,0), 0);
    ballCollider->friction = friction;
    ballCollider->restitution = restitution;

    delete ballShape;

    return ball;
}

ftBody*  createStaticBox(ftVector2 position, real orientation, ftVector2 halfWidth, real friction) {
    ftPolygon *groundShape = ftPolygon::createBox(-1 * halfWidth, halfWidth);
    ftBody* ground = physicsSystem.createStaticBody(position,orientation);

    ftCollider* collider = physicsSystem.createCollider(ground, groundShape, ftVector2(0,0), 0);
    collider->friction = friction;

    return ground;
}


void Demo1_init() {

    createStaticBox(ftVector2(0,-10), 0, ftVector2(50,10),1);

    createDynamicBox(ftVector2(0,4), ftVector2(0.5,0.5), 200, 1);

}

void Demo2_init() {

    createStaticBox(ftVector2(0,-10),0,ftVector2(50,10), 0.2);
    createStaticBox(ftVector2(-2,11), -0.25, ftVector2(6.5, 0.125), 0.2);
    createStaticBox(ftVector2(5.25,9.5),0, ftVector2(0.125,0.5), 0.2);
    createStaticBox(ftVector2(2,7),0.25, ftVector2(6.5,0.125), 0.2);
    createStaticBox(ftVector2(-5.25, 5.5), 0, ftVector2(0.125,0.5),0.2);
    createStaticBox(ftVector2(-2,3), -0.25, ftVector2(6.5,0.125), 0.2);

    float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
    for (int i = 0; i < 5; ++i) {
        createDynamicBox(ftVector2(-7.5 + 2 * i, 14), ftVector2(0.25,0.25), 25, friction[i]);
    }
}

float Random(float lo, float hi)
{
    float r = (float)rand();
    r /= RAND_MAX;
    r = (hi - lo) * r + lo;
    return r;
}

void Demo3_init() {

    createStaticBox(ftVector2(0,-10),0,ftVector2(50,10),0.2);

    for (int i = 0; i < 10; ++i)
    {
        float x = Random(-0.1f, 0.1f);
        createDynamicBox(ftVector2(x, 0.51f + 1.05f * i), ftVector2(0.5,0.5), 1, 0.2);
    }
}

void Demo4_init() {

    createStaticBox(ftVector2(0,0), 0,ftVector2(50,0.5), 0.6);
    createStaticBox(ftVector2(-50,25), 0,ftVector2(0.5,25), 0.2);
    createStaticBox(ftVector2(50,25), 0,ftVector2(0.5,25), 0.2);

    ftVector2 x(-30, 0.75);
    ftVector2 y;

    for (int i = 0; i < 60 ; ++i) {
        y = x;
        for (int j = i; j < 60; ++j) {
            createDynamicBox(y, ftVector2(0.5, 0.5), 10, 0);
            y += ftVector2(1.125f, 0.0f);
        }
        x += ftVector2(0.5625f, 2);
    }

}

void Demo5_init() {

    ftBody* body1 = createStaticBox(ftVector2(0,-10),0, ftVector2(50,10), 0.2);
    ftBody* body2 = createDynamicBox(ftVector2(0,1),ftVector2(6,0.125), 100, 0.2);
    createDynamicBox(ftVector2(-5,2), ftVector2(0.125,0.125), 25, 0.2);
    createDynamicBox(ftVector2(-5.5,2), ftVector2(0.125,0.125), 25, 0.2);
    createDynamicBox(ftVector2(5.5,15), ftVector2(0.5,0.5), 100, 0.2);

    physicsSystem.createHingeJoint(body1, body2, ftVector2(0, 1));
}

void Demo6_init() {

    createStaticBox(ftVector2(0, -10),0, ftVector2(50, 10), 0.2);
    real restitution[8] = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    for (uint32 i = 0; i < 8; ++i) {
        createBall(ftVector2(-7.5 + (2 * i), 15), 50, 0.5, 0.2, restitution[i]);
    }
}

//sparse pyramid
void Demo7_init() {

    int32 spacing = 200;
    for (int k = 0 ; k < 10; ++k) {
        for (int l = 0 ; l < 10; ++l) {
            createStaticBox(ftVector2(-3 + spacing * l, spacing * k), 0,ftVector2(6,0.1), 0.2);
            ftVector2 x(-6 + spacing * l, 0.75 + spacing * k);
            ftVector2 y;

            for (int i = 0; i < 7; ++i) {
                y = x;
                for (int j = i; j < 7; ++j) {
                    createDynamicBox(y, ftVector2(0.5, 0.5), 10, 0.2);
                    y += ftVector2(1.125f, 0.0f);
                }
                x += ftVector2(0.5625f, 2.0f);
            }

        }
    }

}


void Demo8_init() {

    createStaticBox(ftVector2(-2,11), -0.25, ftVector2(6.5, 0.125), 0.2);
    createStaticBox(ftVector2(5.25,9.5),0, ftVector2(0.125,0.5), 0.2);
    createStaticBox(ftVector2(2,7),0.25, ftVector2(6.5,0.125), 0.2);
    createStaticBox(ftVector2(-5.25, 5.5), 0, ftVector2(0.125,0.5),0.2);
    createStaticBox(ftVector2(-2,3), -0.25, ftVector2(6.5,0.125), 0.2);

    float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
    for (int i = 0; i < 5; ++i) {
        createBall(ftVector2(-7.5 + 2 * i, 14), 10, 0.2, friction[i], 0);
    }


    ftVector2 pivot1 = ftVector2(5,1);
    ftVector2 pivot2 = ftVector2(9,1);


    createStaticBox(ftVector2(0,-11),0, ftVector2(50,10), 0.2);
    ftBody* body2 = createDynamicBox(ftVector2(7,1),ftVector2(4,0.25), 100, 0.2);
    body2->colliders->group = 1;

    ftBody* body3 = createBall(pivot1,10, 0.5,0.2, 0);
    body3->colliders->group = 1;
    ftBody* body4 = createBall(pivot2,10, 0.5,0.2, 0);
    body4->colliders->group = 1;

    physicsSystem.createHingeJoint(body2, body3, pivot1);
    physicsSystem.createHingeJoint(body2, body4, pivot2);
}

void createPendulum(ftVector2 position) {
    ftBody* ceiling = createStaticBox(position, 0, ftVector2(0.5,0.5), 0.6);
    ftBody* ball = createBall(ftVector2(position.x - 5, position.y), 1, 0.5, 0.2, 0.88);
    physicsSystem.createDistanceJoint(ceiling, ball, ftVector2(0,0), ftVector2(0.4,0));
}

//distance joint
void Demo9_init() {

    int nPendulum = 6;
    real spacing = 0.001;
    int leftPendulum = nPendulum / 2;
    ftVector2 position(- leftPendulum * (1.0 + spacing * 2), 10);
    createPendulum(position);

    for (int i = 1; i < nPendulum; ++i) {
        position += ftVector2(1.0 + spacing, 0);
        ftBody* ceiling = createStaticBox(position, 0, ftVector2(0.5,0.5), 0.6);
        ftBody* ball = createBall(ftVector2(position.x, position.y - 5), 1, 0.5, 0.2, 0.88);
        physicsSystem.createDistanceJoint(ceiling, ball, ftVector2(0,0), ftVector2(0,0.4));
    }
}

//spring
void Demo10_init() {
    createStaticBox(ftVector2(0, -10),0, ftVector2(50, 10), 0.2);
    ftBody* body1 = createStaticBox(ftVector2(10,0.5), 0, ftVector2(0.5,0.5), 0.2);
    ftBody* body2 = createDynamicBox(ftVector2(0,0.5),ftVector2(0.5,0.5001), 10, 0.5);

    ftBody* body3 = createDynamicBox(ftVector2(-10,0.5),ftVector2(0.5,0.5), 100, 0.5);
    body3->velocity.x = 30;

    ftSpringJoint* joint = physicsSystem.createSpringJoint(body1, body2, ftVector2(0,0), ftVector2(0,0));;
    joint->stiffness = 1000;
}

//liquid
void Demo11_init() {

    createStaticBox(ftVector2(0,0), 0,ftVector2(50,0.5), 0.6);
    createStaticBox(ftVector2(-50,50), 0,ftVector2(0.5,50), 0.2);
    createStaticBox(ftVector2(50,50), 0,ftVector2(0.5,50), 0.2);

    ftVector2 x(-30, 0.75);
    ftVector2 y;

    for (int i = 0; i < 60 ; ++i) {
        y = x;
        for (int j = 0; j < 30; ++j) {
            ftBody* body = createBall(y, 1, 0.5, 0, 0.6);
            body->userdata = (void*)(i*j);
            y += ftVector2(1.125f, 0.0f);
        }
        x += ftVector2(0.5625f, 2);
    }

}

/// dominoes and motor
void Demo12_init() {
    camera.centerX = 0;
    camera.centerY = 25;
    ftBody* pendulumCeiling = createStaticBox(ftVector2(-40,50), 0, ftVector2(0.5,0.5), 0.2);
    ftBody* ground = createStaticBox(ftVector2(0,0), 0,ftVector2(50,0.5), 0.2);

    ftBody* pendulumBall = createBall(ftVector2(-45, 50), 5, 0.5, 0.2, 0.2);
    createStaticBox(ftVector2(-22,39.5), 0, ftVector2(20,0.5), 0.8);
    for (int i = 0 ; i < 20; ++i) {
        createDynamicBox(ftVector2(-40.5 + i * 2, 42.5), ftVector2(0.5,2.5), 10, 0.4);
    }

    createStaticBox(ftVector2(8,32.5), - PI / 8, ftVector2(10,0.5), 0.2);

    {
        ftBody *body2 = createDynamicBox(ftVector2(7, 3), ftVector2(4, 0.25), 100, 0.2);
        body2->colliders->group = 1;

        ftVector2 pivot1 = ftVector2(5, 1);
        ftVector2 pivot2 = ftVector2(9, 1);
        ftBody *body3 = createBall(pivot1, 10, 0.5, 0.2, 0);
        body3->colliders->group = 1;
        ftBody *body4 = createBall(pivot2, 10, 0.5, 0.2, 0);
        body4->colliders->group = 1;

        ftBody* body5 = createDynamicBox(pivot1, ftVector2(0.05,0.05), 100, 0.2);
        body5->colliders->group = 1;
        ftBody* body6 = createDynamicBox(pivot2, ftVector2(0.05,0.05), 100, 0.2);
        body6->colliders->group = 1;

        physicsSystem.createHingeJoint(body5, body3, pivot1);
        physicsSystem.createHingeJoint(body6, body4, pivot2);
        physicsSystem.createDynamoJoint(body5, body3, -1000, -100);
        physicsSystem.createDynamoJoint(body6, body4, -1000, -100);
        ftSpringJoint* spring1 = physicsSystem.createSpringJoint(body2, body5, ftVector2(-1.5,0), ftVector2(0,0));
        ftSpringJoint* spring2 = physicsSystem.createSpringJoint(body2, body6, ftVector2(1.5,0), ftVector2(0,0));

    }

    physicsSystem.createDistanceJoint(pendulumCeiling, pendulumBall, ftVector2(0,0), ftVector2(0,0));
}

//obj different size
void Demo13_init() {
    createStaticBox(ftVector2(0,0), 0,ftVector2(120,0.5), 0.6);
    createStaticBox(ftVector2(-122,50), 0,ftVector2(2,50), 0.2);
    createStaticBox(ftVector2(122,50), 0,ftVector2(2,50), 0.2);

    ftVector2 x(-30, 5.75);
    ftVector2 y;

    real density = 1;

    createDynamicBox(ftVector2(0,250), ftVector2(50, 50), 10000 * density,  0.2);
    createDynamicBox(ftVector2(-100,250), ftVector2(20, 20),1600 * density,  0.2);

    std::srand(0);
    for (int i = 0; i < 50 ; ++i) {
        float maxWidth = 0.0f;
        y = x;
        for (int j = i; j < 30; ++j) {
            int random = std::rand() % 2;
            float width = 5;
            if (random == 0) {
                width = 2;
            }
            float halfWidth = width/2;
            y += ftVector2(halfWidth, 0.0f);
            createDynamicBox(y, ftVector2(halfWidth, halfWidth), width * width * density, 0.4);
            y += ftVector2(halfWidth, 0.0f);
            if (maxWidth < width) maxWidth = width;
        }
        x += ftVector2(0.5625, maxWidth + 0.25);
    }

}



void Demo14_init() {

    int nPendulum = 6;
    real spacing = 0.001;
    int leftPendulum = nPendulum / 2;
    ftVector2 position(- leftPendulum * (1.0 + spacing * 2) + 25, 15);
    createPendulum(position);

    for (int i = 1; i < nPendulum; ++i) {
        position += ftVector2(1.0 + spacing, 0);
        ftBody *ceiling = createStaticBox(position, 0, ftVector2(0.5, 0.5), 0.6);
        ftBody *ball = createBall(ftVector2(position.x, position.y - 5), 1, 0.5, 0.2, 0.88);
        physicsSystem.createDistanceJoint(ceiling, ball, ftVector2(0, 0), ftVector2(0, 0.4));
    }

    for (int i = 0; i < 30; ++i) {
        real xOffset = i * 160;
        createStaticBox(ftVector2(xOffset + 25, 4), 0, ftVector2(25, 4), 0.6);
        createStaticBox(ftVector2(xOffset + 60, 10), 0, ftVector2(10, 10), 0.6);
        createStaticBox(ftVector2(xOffset + 80, 20), 0, ftVector2(10, 20), 0.6);
        createStaticBox(ftVector2(xOffset + 100, 30), 0, ftVector2(10, 30), 0.6);
        createStaticBox(ftVector2(xOffset + 140, 56), 0, ftVector2(10, 4), 0.6);
    }
}

//test liquid
void TestCase1_init() {
    createStaticBox(ftVector2(0,0), 0,ftVector2(50,0.5), 0.6);
    createStaticBox(ftVector2(-50,25), 0,ftVector2(0.5,25), 0.2);
    createStaticBox(ftVector2(50,25), 0,ftVector2(0.5,25), 0.2);

    ftVector2 x(-30, 0.75);
    ftVector2 y;

    for (int i = 0; i < 60 ; ++i) {
        y = x;
        for (int j = 0; j < 30; ++j) {
            ftBody* body = createBall(y, 1, 0.5, 0, 0.6);
            body->userdata = (void*)(i*j);
            y += ftVector2(1.125f, 0.0f);
        }
        x += ftVector2(0.5625f, 2);
    }
}

//test sparse pyramid
void TestCase2_init() {
    int32 spacing = 500;
    for (int k = 0 ; k < 8; ++k) {
        for (int l = 0 ; l < 8; ++l) {
            createStaticBox(ftVector2(-3 + spacing * l, spacing * k), 0,ftVector2(6,0.1), 0.2);
            ftVector2 x(-6 + spacing * l, 0.75 + spacing * k);
            ftVector2 y;

            for (int i = 0; i < 7; ++i) {
                y = x;
                for (int j = i; j < 7; ++j) {
                    createDynamicBox(y, ftVector2(0.5, 0.5), 10, 0.2);
                    y += ftVector2(1.125f, 0.0f);
                }
                x += ftVector2(0.5625f, 2.0f);
            }

        }
    }
}

//obj different size
void TestCase3_init() {
    createStaticBox(ftVector2(0,0), 0,ftVector2(120,0.5), 0.6);
    createStaticBox(ftVector2(-122,50), 0,ftVector2(2,50), 0.2);
    createStaticBox(ftVector2(122,50), 0,ftVector2(2,50), 0.2);

    ftVector2 x(-30, 0.75);
    ftVector2 y;

    real density = 1;

    createDynamicBox(ftVector2(0,250), ftVector2(50, 50), 10000 * density,  0.2);
    createDynamicBox(ftVector2(-100,250), ftVector2(20, 20),1600 * density,  0.2);

    std::srand(0);
    for (int i = 0; i < 50 ; ++i) {
        float maxWidth = 0.0f;
        y = x;
        for (int j = i; j < 30; ++j) {
            int random = std::rand() % 2;
            float width = 5;
            if (random == 0) {
                width = 2;
            }
            float halfWidth = width/2;
            y += ftVector2(halfWidth, 0.0f);
            createDynamicBox(y, ftVector2(halfWidth, halfWidth), width * width * density, 0.4);
            y += ftVector2(halfWidth, 0.0f);
            if (maxWidth < width) maxWidth = width;
        }
        x += ftVector2(0.5625, maxWidth + 0.25);
    }
}

//platformer
void TestCase4_init() {

    int nPendulum = 6;
    real spacing = 0.001;
    int leftPendulum = nPendulum / 2;
    ftVector2 position(- leftPendulum * (1.0 + spacing * 2), 10);
    createPendulum(position);

    for (int i = 1; i < nPendulum; ++i) {
        position += ftVector2(1.0 + spacing, 0);
        ftBody *ceiling = createStaticBox(position, 0, ftVector2(0.5, 0.5), 0.6);
        ftBody *ball = createBall(ftVector2(position.x, position.y - 5), 1, 0.5, 0.2, 0.88);
        physicsSystem.createDistanceJoint(ceiling, ball, ftVector2(0, 0), ftVector2(0, 0.4));
    }

    for (int i = 0; i < 30; ++i) {
        real xOffset = i * 160;
        createPendulum(ftVector2(xOffset + 25, 15));
        createStaticBox(ftVector2(xOffset + 25, 4), 0, ftVector2(25, 4), 0.6);
        createStaticBox(ftVector2(xOffset + 60, 10), 0, ftVector2(10, 10), 0.6);
        createPendulum(ftVector2(xOffset + 80, 47));
        createStaticBox(ftVector2(xOffset + 80, 20), 0, ftVector2(10, 20), 0.6);
        createStaticBox(ftVector2(xOffset + 100, 30), 0, ftVector2(10, 30), 0.6);
        createStaticBox(ftVector2(xOffset + 140, 56), 0, ftVector2(10, 4), 0.6);
    }
}
#endif //FALTON_DEMO_H
