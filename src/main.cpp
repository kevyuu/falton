// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>

// Include falton
#include <falton/math/math.h>

#include <allegro5/allegro5.h>
#include <allegro5/allegro_primitives.h>
#include <falton/physics/shape/ftPolygon.h>
#include <falton/physics/ftMassComputer.h>
#include <stdlib.h>

#include <iostream>
#include <falton/physics/dynamic/ftBody.h>
#include <falton/physics/dynamic/ftPhysicsSystem.h>

using namespace std;

bool done;
ALLEGRO_EVENT_QUEUE* event_queue;
ALLEGRO_TIMER* timer;
ALLEGRO_DISPLAY* display;

ALLEGRO_COLOR electricBlue;

ftPhysicsSystem* physicsSystem;
ftChunkArray<ftCollider*> colliders(64);

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 800;

void abort_game(const char* message)
{
    printf("%s \n", message);
    exit(1);
}

void allegro_init(void)
{
    if (!al_init())
        abort_game("Failed to initialize allegro");

    if (!al_init_primitives_addon())
        abort_game("Failed to initialize primitive addon");

    if (!al_install_keyboard())
        abort_game("Failed to install keyboard");

    timer = al_create_timer(1.0 / 60);
    if (!timer)
        abort_game("Failed to create timer");

    al_set_new_display_flags(ALLEGRO_WINDOWED);
    display = al_create_display(SCREEN_WIDTH, SCREEN_HEIGHT);
    if (!display)
        abort_game("Failed to create display");

    event_queue = al_create_event_queue();
    if (!event_queue)
        abort_game("Failed to create event queue");

    al_register_event_source(event_queue, al_get_keyboard_event_source());
    al_register_event_source(event_queue, al_get_timer_event_source(timer));
    al_register_event_source(event_queue, al_get_display_event_source(display));

    electricBlue = al_map_rgb(44,177,255);

    done = false;
}

void createDynamicBox(ftVector2 position, ftVector2 halfWidth, real mass, real friction) {
    ftPolygon *boxShape = ftPolygon::createBox(-1 * halfWidth, halfWidth);
    ftMassProperty boxmp = ftMassComputer::computeForPolygon(*boxShape,mass, ftVector2(0,0));
    ftBodyDef boxDef;
    boxDef.position = position;
    boxDef.orientation = 0;
    boxDef.centerOfMass = boxmp.centerOfMass;
    boxDef.mass = boxmp.mass;
    boxDef.moment = boxmp.moment;
    boxDef.bodyType = DYNAMIC;
    ftBody* box = physicsSystem->createBody(boxDef);

    ftColliderDef boxColDef;
    boxColDef.body = box;
    boxColDef.shape = boxShape;
    boxColDef.orientation = 0;
    boxColDef.position = ftVector2(0,0);
    boxColDef.friction = friction;
    ftCollider* boxCollider = physicsSystem->createCollider(boxColDef);
    colliders.push(boxCollider);
}

void createStaticBox(ftVector2 position, real orientation, ftVector2 halfWidth, real friction) {
    ftPolygon *groundShape = ftPolygon::createBox(-1 * halfWidth, halfWidth);
    ftBodyDef groundDef;
    groundDef.bodyType = STATIC;
    groundDef.position = position;
    groundDef.orientation = orientation;
    ftBody* ground = physicsSystem->createBody(groundDef);

    ftColliderDef colliderDef;
    colliderDef.body = ground;
    colliderDef.friction = friction;
    colliderDef.shape = groundShape;
    colliderDef.position = ftVector2(0,0);
    ftCollider* groundCollider = physicsSystem->createCollider(colliderDef);
    colliders.push(groundCollider);
}

void Demo1_init() {
    physicsSystem = new ftPhysicsSystem;
    physicsSystem->init(ftVector2(0,-10));

    createStaticBox(ftVector2(0,-10), 0, ftVector2(50,10),1);

    createDynamicBox(ftVector2(0,4), ftVector2(0.5,0.5), 200, 1);

}

void Demo2_init() {
    physicsSystem = new ftPhysicsSystem;
    physicsSystem->init(ftVector2(0,-10));

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
    physicsSystem = new ftPhysicsSystem;
    physicsSystem->init(ftVector2(0,-10));

    createStaticBox(ftVector2(0,-10),0,ftVector2(50,10),0.2);

    for (int i = 0; i < 10; ++i)
    {
        float x = Random(-0.1f, 0.1f);
        createDynamicBox(ftVector2(x, 0.51f + 1.05f * i), ftVector2(0.5,0.5), 1, 0.2);
    }
}


void init() {
    allegro_init();
    Demo2_init();
}

void shutdown(void)
{
    if (timer)
        al_destroy_timer(timer);

    if (display)
        al_destroy_display(display);

    if (event_queue)
        al_destroy_event_queue(event_queue);
}

ftVector2 spaceToView(ftVector2 spaceVector) {

    ftVector2 viewVector;
    real scale = 40;
    viewVector.x = SCREEN_WIDTH / 2 + (spaceVector.x * scale);
    viewVector.y = SCREEN_HEIGHT / 2 - ((spaceVector.y - 7) * scale);

    return viewVector;
}

void draw_polygon(const ftTransform& transform, ftPolygon* polygon, ALLEGRO_COLOR *color) {

    for (int i=0;i<polygon->numVertex;++i) {
        int i1 = i;
        int i2 = (i + 1) % polygon->numVertex;

        ftVector2 vert1 = spaceToView(transform * polygon->vertices[i1]);
        ftVector2 vert2 = spaceToView(transform * polygon->vertices[i2]);

        al_draw_line(vert1.x, vert1.y,
                     vert2.x, vert2.y,
                     *color, 1.0);
    }
}


void draw_collider(ftCollider* collider) {
    draw_polygon(collider->body->transform, (ftPolygon*)collider->shape, &electricBlue);
}

void update_logic() {
    physicsSystem->step(1.0/60);
}

void update_graphics() {
    for (uint32 i=0;i<colliders.getSize();i++) {
        draw_collider(colliders[i]);
    }
}

void game_loop(void)
{
    bool redraw = true;
    al_start_timer(timer);

    while (!done) {
        ALLEGRO_EVENT event;
        al_wait_for_event(event_queue, &event);

        if (event.type == ALLEGRO_EVENT_TIMER) {
            redraw = true;
            update_logic();
        }
        else if (event.type == ALLEGRO_EVENT_KEY_DOWN) {
            if (event.keyboard.keycode == ALLEGRO_KEY_ESCAPE) {
                done = true;
            }
            //get_user_input();
        }

        if (redraw && al_is_event_queue_empty(event_queue)) {
            redraw = false;
            al_clear_to_color(al_map_rgb(0, 0, 0));
            update_graphics();
            al_flip_display();
        }
    }
}

int main(int argc __attribute__((unused)), char* argv[] __attribute((unused)))
{
    init();
    game_loop();
    shutdown();
    return 0;
}