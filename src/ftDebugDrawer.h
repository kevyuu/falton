//
// Created by Kevin Yu on 2016-05-13.
//

#ifndef FALTON_FTDEBUGDRAWER_H
#define FALTON_FTDEBUGDRAWER_H

#include <allegro5/allegro5.h>
#include <allegro5/allegro_primitives.h>

#include <falton/physics/dynamic/ftPhysicsSystem.h>
#include <falton/physics/shape/ftPolygon.h>
#include <falton/physics/shape/ftCircle.h>

class ftDebugDrawer {

    ALLEGRO_COLOR defaultColor;

    ALLEGRO_COLOR staticColor;
    ALLEGRO_COLOR dynamicColor;
    ALLEGRO_COLOR kinematicColor;
    ALLEGRO_COLOR sleepColor;

public:

    void init() {
        dynamicColor = al_map_rgb(255, 44, 177);
        kinematicColor = al_map_rgb(177,255, 44);
        staticColor = al_map_rgb(44,177,25);
        sleepColor = al_map_rgb(200,200,200);
        defaultColor = al_map_rgb(44,177,255);
    }

    ALLEGRO_COLOR getBodyColor(ftBody* body) {
        if (body->activationState == SLEEP) return sleepColor;
        switch(body->bodyType) {
            case STATIC : return staticColor;
            case DYNAMIC : return dynamicColor;
            case KINEMATIC : return kinematicColor;
        }
        return defaultColor;
    }

    void draw(ftPhysicsSystem* world) {
        world->iterateBody(&drawFunc, this);
    }

    static void drawFunc(ftBody* body, void* ptr) {
        ftDebugDrawer* drawer = (ftDebugDrawer*) ptr;
        ALLEGRO_COLOR color = drawer->getBodyColor(body);
        for (ftCollider* collider = body->colliders;
             collider != nullptr; collider = collider->next) {
            ftTransform transform = body->transform * collider->transform;
            ftShape* shape = collider->shape;
            if (shape->shapeType == SHAPE_CIRCLE) {
                drawCircle(transform, (ftCircle*) shape, &color);
            } else {
                drawPolygon(transform, (ftPolygon*) shape, &color);
            }
        }
    }


    static ftVector2 spaceToView(ftVector2 spaceVector) {

        ftVector2 viewVector;
        real scale = 20;
        viewVector.x = 800 / 2 + (spaceVector.x * scale);
        viewVector.y = 800 / 2 - ((spaceVector.y - 7) * scale);

        return viewVector;
    }


    static void drawPolygon(const ftTransform& transform, ftPolygon* polygon, ALLEGRO_COLOR *color) {

        for (uint32 i=0;i<polygon->numVertex;++i) {
            int i1 = i;
            int i2 = (i + 1) % polygon->numVertex;

            ftVector2 vert1 = spaceToView(transform * polygon->vertices[i1]);
            ftVector2 vert2 = spaceToView(transform * polygon->vertices[i2]);

            al_draw_line(vert1.x, vert1.y,
                         vert2.x, vert2.y,
                         *color, 1.0);
        }
    }

    static void drawCircle(const ftTransform& transform, ftCircle* circle, ALLEGRO_COLOR * color) {

        ftVector2 spaceCenter = spaceToView(transform.center);

        al_draw_circle(spaceCenter.x,spaceCenter.y,circle->radius * 20, *color,1.0);
        ftVector2 one(1,0);
        ftVector2 boundary = transform.center + (transform.rotation * one) * circle->radius;
        boundary = spaceToView(boundary);
        al_draw_line(spaceCenter.x, spaceCenter.y,
                     boundary.x, boundary.y, *color, 1.0);
    }

};


#endif //FALTON_FTDEBUGDRAWER_H
