//
// Created by Kevin Yu on 2016-05-19.
//

#ifndef FALTON_DRAWPHYSICS_H
#define FALTON_DRAWPHYSICS_H

#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <falton/physics/dynamic/ftPhysicsSystem.h>
#include <falton/physics/shape/ftPolygon.h>
#include <falton/physics/shape/ftCircle.h>

#include <iostream>
using namespace std;

extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;

struct DrawConfig {
    ImColor staticBorder = ImColor(225,10,10);
    ImColor dynamicBorder = ImColor(10,255,10);
    ImColor kinematicBorder = ImColor(10,10,255);
    ImColor sleepingBorder = ImColor(255,255,255);
    ImColor contactBorder;

    bool showStatic = true;
    bool showDynamic = true;
    bool showKinematic = true;
    bool showSleeping = true;
    bool showContact;
};

struct Camera {
    float centerX;
    float centerY;
    float halfWidthX;
    float halfWidthY;

    void zoom(float z) {
        halfWidthX /= z;
        halfWidthY /= z;
    }
};


ftVector2 spaceToViewCoordinate(ftVector2 spaceVector, Camera& camera) {

    ftVector2 viewVector;
    float scaleX = SCREEN_WIDTH / (2 * camera.halfWidthX);
    float scaleY = SCREEN_HEIGHT / (2 * camera.halfWidthY);

    float minX = camera.centerX - camera.halfWidthX;
    float minY = camera.centerY - camera.halfWidthY;

    viewVector.x = (spaceVector.x - minX) * scaleX;
    viewVector.y = SCREEN_HEIGHT - ((spaceVector.y - minY) * scaleY);

    return viewVector;
}

ftVector2 spaceToViewDirection(ftVector2 spaceVector, Camera& camera) {
    ftVector2 viewVector;

    float scaleX = SCREEN_WIDTH / (2 * camera.halfWidthX);
    float scaleY = SCREEN_HEIGHT / (2 * camera.halfWidthY);

    viewVector.x = spaceVector.x * scaleX;
    viewVector.y = spaceVector.y * scaleY;

    return viewVector;
}

real spaceToViewLength(float length, Camera& camera) {
    real screenLength = real_sqrt(SCREEN_WIDTH * SCREEN_WIDTH + SCREEN_HEIGHT * SCREEN_HEIGHT);
    real worldLength = real_sqrt(camera.halfWidthX * camera.halfWidthX * 4 +  camera.halfWidthY * camera.halfWidthY * 4);
    return length * screenLength / worldLength;
}

void draw_polygon(const ftTransform& transform, Camera& camera, ftPolygon* polygon, ALLEGRO_COLOR *color) {

    for (uint32 i=0;i<polygon->numVertex;++i) {
        int i1 = i;
        int i2 = (i + 1) % polygon->numVertex;

        ftVector2 vert1 = spaceToViewCoordinate(transform * polygon->vertices[i1], camera);
        ftVector2 vert2 = spaceToViewCoordinate(transform * polygon->vertices[i2], camera);

        al_draw_line(vert1.x, vert1.y,
                     vert2.x, vert2.y,
                     *color, 1.0);
    }
}

void draw_circle(const ftTransform& transform, Camera& camera, ftCircle* circle, ALLEGRO_COLOR * color) {

    ftVector2 spaceCenter = spaceToViewCoordinate(transform.center, camera);

    al_draw_circle(spaceCenter.x,spaceCenter.y,spaceToViewLength(circle->radius, camera), *color,1.0);
    ftVector2 one(1,0);
    ftVector2 boundary = transform.center + (transform.rotation * one) * circle->radius;
    boundary = spaceToViewCoordinate(boundary,camera);
    al_draw_line(spaceCenter.x, spaceCenter.y,
                 boundary.x, boundary.y, *color, 1.0);
}


void draw_collider(ftCollider* collider, ImColor& color, Camera& camera) {
    ALLEGRO_COLOR alColor;
    alColor.r = color.Value.x;
    alColor.g = color.Value.y;
    alColor.b = color.Value.z;
    alColor.a = color.Value.w;
    if (collider->shape->shapeType == SHAPE_CIRCLE) {
        draw_circle(collider->body->transform, camera, (ftCircle*)collider->shape, &alColor);
    } else {
        draw_polygon(collider->body->transform, camera, (ftPolygon *) collider->shape, &alColor);
    }
}


void drawPhysics(ftPhysicsSystem* physicsSystem, DrawConfig drawConfig, Camera& camera) {

    if (drawConfig.showKinematic) {
        auto drawKinematic = [&drawConfig,&camera](ftBody* body) -> void {
            for (ftCollider* collider = body->colliders; collider != nullptr; collider = collider->next) {
                draw_collider(collider, drawConfig.kinematicBorder, camera);
            }
        };
        physicsSystem->forEachKinematicBody(drawKinematic);
    }

    if (drawConfig.showStatic) {
        auto drawStatic = [&drawConfig,&camera](ftBody* body) -> void {
            for (ftCollider* collider = body->colliders; collider != nullptr; collider = collider->next) {
                draw_collider(collider, drawConfig.staticBorder, camera);
            }
        };
        physicsSystem->forEachStaticBody(drawStatic);
    }

    auto drawDynamic = [&drawConfig,&camera](ftBody* body) -> void {
        for (ftCollider* collider = body->colliders; collider != nullptr; collider = collider->next) {
            if (body->activationState == SLEEP) {
                if (drawConfig.showSleeping) draw_collider(collider, drawConfig.sleepingBorder, camera);
            } else {
                if (drawConfig.showDynamic) draw_collider(collider, drawConfig.dynamicBorder, camera);
            }
        }
    };
    physicsSystem->forEachDynamicBody(drawDynamic);


}



#endif //FALTON_DRAWPHYSICS_H
