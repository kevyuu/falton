// ImGui - standalone example application for Allegro 5
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.

#include <stdint.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_a5.h"
#include "imgui/imgui_internal.h"
#include "Demo.h"
#include "window/DemoWindow.h"
#include "DrawPhysics.h"
#include "window/ExecutionButton.h"
#include "window/PhysicsConfigWindow.h"
#include "window/PerformanceWindow.h"
#include "window/DrawerConfigWindow.h"

#include <iostream>
#include <falton/physics/collision/broadphase/ftNSquaredBroadphase.h>
#include <falton/physics/collision/broadphase/ftDynamicBVH.h>
#include <falton/physics/collision/broadphase/ftHierarchicalGrid.h>
#include <falton/physics/collision/broadphase/ftToroidalGrid.h>

using namespace std;


ALLEGRO_TIMER* timer;
ALLEGRO_EVENT_QUEUE* eventQueue;
ALLEGRO_DISPLAY* display;

int SCREEN_WIDTH = 1280;
int SCREEN_HEIGHT = 720;

ftPhysicsSystem physicsSystem;
ftBroadphaseSystem* broadphaseSystem;
PhysicsConfigWindow physicsConfigWindow;
DemoWindow demoWindow;
PerformanceWindow performanceWindow;
DrawerConfigWindow drawConfigWindow;

Camera camera;


void gameLoop();
void updateLogic();
void updateGraphics();
void updateImGUI();
void cleanUp();

long getMicro() {
    timeval time;
    gettimeofday(&time, NULL);
    long micro = (time.tv_sec * 1000000) + (time.tv_usec);
    return micro;
}

void abort_game(const char* message)
{
    printf("%s \n", message);
    exit(1);
}

ExecutionButton executionButton;

int main(int argc, char *argv[])
{
    // Setup Allegro
    al_init();
    al_install_keyboard();
    al_install_mouse();
    al_init_primitives_addon();
    al_set_new_display_flags(ALLEGRO_RESIZABLE);
    display = al_create_display(SCREEN_WIDTH, SCREEN_HEIGHT);
    al_set_window_title(display, "ImGui Allegro 5 example");

    timer = al_create_timer(1.0/60);
    if (!timer) {
        abort_game("Failed to create timer");
    }

    eventQueue = al_create_event_queue();
    al_register_event_source(eventQueue, al_get_timer_event_source(timer));
    al_register_event_source(eventQueue, al_get_display_event_source(display));
    al_register_event_source(eventQueue, al_get_keyboard_event_source());
    al_register_event_source(eventQueue, al_get_mouse_event_source());

    // Setup ImGui binding
    ImGui_ImplA5_Init(display);

    camera.centerX = 0;
    camera.centerY = 0;
    camera.halfWidthX = SCREEN_WIDTH/24;
    camera.halfWidthY = SCREEN_HEIGHT/24;

    // Load Fonts
    // (there is a default font, this is only if you want to change it. see extra_fonts/README.txt for more details)
    //ImGuiIO& io = ImGui::GetIO();
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("../../extra_fonts/Cousine-Regular.ttf", 15.0f);
    //io.Fonts->AddFontFromFileTTF("../../extra_fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../extra_fonts/ProggyClean.ttf", 13.0f);
    //io.Fonts->AddFontFromFileTTF("../../extra_fonts/ProggyTiny.ttf", 10.0f);
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());


    gameLoop();

    cleanUp();

    return 0;
}

void gameLoop(void)
{
    bool redraw = true;
    bool running = true;
    al_start_timer(timer);

    while (running) {
        ALLEGRO_EVENT event;
        al_wait_for_event(eventQueue, &event);
        ImGui_ImplA5_ProcessEvent(&event);
        if (event.type == ALLEGRO_EVENT_TIMER) {
            redraw = true;
            updateLogic();
        } else if (event.type == ALLEGRO_EVENT_KEY_DOWN) {
            switch (event.keyboard.keycode) {
                case ALLEGRO_KEY_J : {
                    camera.zoom(2);
                    break;
                }
                case ALLEGRO_KEY_L : {
                    camera.zoom(0.5f);
                    break;
                }
                case ALLEGRO_KEY_DOWN : {
                    camera.centerY -= 5;
                    break;
                }
                case ALLEGRO_KEY_UP : {
                    camera.centerY += 5;
                    break;
                }
                case ALLEGRO_KEY_LEFT : {
                    camera.centerX -= 5;
                    break;
                }
                case ALLEGRO_KEY_RIGHT : {
                    camera.centerX += 5;
                    break;
                }
            }

        } else if (event.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            running = false;
        } else if (event.type == ALLEGRO_EVENT_DISPLAY_RESIZE)
        {
            ImGui_ImplA5_InvalidateDeviceObjects();
            al_acknowledge_resize(display);
            Imgui_ImplA5_CreateDeviceObjects();
        }

        if (redraw && al_is_event_queue_empty(eventQueue)) {
            redraw = false;
            updateImGUI();
            al_clear_to_color(al_map_rgb(40, 40, 40));
            updateGraphics();
            al_flip_display();
        }

    }
}

void initDemo(int demoNum) {
    switch(demoNum) {
        case 0 :
            Demo1_init();
            break;
        case 1 :
            Demo2_init();
            break;
        case 2 :
            Demo3_init();
            break;
        case 3 :
            Demo4_init();
            break;
        case 4 :
            Demo5_init();
            break;
        case 5 :
            Demo6_init();
            break;
        case 6 :
            Demo7_init();
            break;
        case 7 :
            Demo8_init();
            break;
        case 8 :
            Demo9_init();
            break;
    }
}

ftBroadphaseSystem* constructBroadphase() {
    switch (physicsConfigWindow.broadphase) {
        case PhysicsConfigWindow::NSquaredBroadphase : {
            return new ftNSquaredBroadphase();
        }
        case PhysicsConfigWindow::DynamicBVH : {
            ftDynamicBVH* bvh =  new ftDynamicBVH();
            bvh->setConfiguration(physicsConfigWindow.bvhConfig);
            return bvh;
        }
        case PhysicsConfigWindow::HierarchicalGrid : {
            ftHierarchicalGrid* grid = new ftHierarchicalGrid();
            grid->setConfiguration(physicsConfigWindow.hierarchicalConfig);
            return grid;
        }
        case PhysicsConfigWindow::ToroidalGrid : {
            ftToroidalGrid* grid = new ftToroidalGrid();
            grid->setConfiguration(physicsConfigWindow.toroidalConfig);
            return grid;
        }
        case PhysicsConfigWindow::QuadTree : {
            ftQuadTree* quadTree = new ftQuadTree();
            quadTree->setConfiguration(physicsConfigWindow.quadConfig);
            return quadTree;
        }
    }
    return nullptr;
}

void updateLogic() {
    if (executionButton.starting) {
        broadphaseSystem = constructBroadphase();
        physicsSystem.installBroadphase(broadphaseSystem);
        physicsSystem.init();
        initDemo(demoWindow.demo);
    }
    else if (executionButton.restarting) {
        physicsSystem.shutdown();
        delete broadphaseSystem;
        broadphaseSystem = constructBroadphase();
        physicsSystem.installBroadphase(broadphaseSystem);
        physicsSystem.setConfiguration(physicsConfigWindow.config);
        physicsSystem.init();
        initDemo(demoWindow.demo);
        ftBenchmark::Clear();
    }
    if (executionButton.stopping) {
        physicsSystem.shutdown();
        delete broadphaseSystem;
        ftBenchmark::Clear();
    }
    if (executionButton.playing) {
        ftBenchmark::BeginFrame();
        performanceWindow.updateStartFrame();
        static int stepIdx = -1;
        stepIdx = ftBenchmark::Begin("ftPhysicsSystem::step",stepIdx);
        physicsSystem.step(1.0/60);
        ftBenchmark::End();
        ftBenchmark::EndFrame();
    }
}

void updateGraphics() {
    ImGui::Render();
    drawPhysics(&physicsSystem, drawConfigWindow.config, camera);
}

void updateImGUI() {

    ImGui_ImplA5_NewFrame();

    static bool open = true;
    ImGui::ShowTestWindow(&open);
    executionButton.draw();
    demoWindow.draw();
    physicsConfigWindow.draw();
    performanceWindow.draw();
    drawConfigWindow.draw();
}

void cleanUp() {
    ImGui_ImplA5_Shutdown();
    al_destroy_event_queue(eventQueue);
    al_destroy_display(display);
}


