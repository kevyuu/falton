#ifndef FALTON_FALTONGUI_H
#define FALTON_FALTONGUI_H

#include "window/DemoWindow.h"
#include "window/ExecutionButton.h"
#include "window/PhysicsConfigWindow.h"
#include "window/PerformanceWindow.h"

#include <iostream>

namespace FaltonGUI {

    struct ftWindowInfo {
        int width;
        int height;
    };

    DemoWindow demoWindow;
    PhysicsConfigWindow configWindow;
    ExecutionButton executionButton;
    PerformanceWindow performanceWindow;
    ftBroadphaseSystem* broadphaseSystem;

    ftDebugCamera debugCamera = {{0.0f, 0.0f}, 1280.0f / 24, 720.0f / 24};
    ftDebugCanvas canvas;

    void Init() {
        canvas.Init();
    }

    void InitAndLoadPhysics(ftPhysicsSystem* physicsSystem) {
        
        switch (configWindow.broadphase) {
            case PhysicsConfigWindow::NSquaredBroadphase : {
                broadphaseSystem = new ftNSquaredBroadphase();
                break;
            }
            case PhysicsConfigWindow::DynamicBVH : {
                ftDynamicBVH* bvh = new ftDynamicBVH();
                bvh->setConfiguration(configWindow.bvhConfig);
                broadphaseSystem = bvh;
                break;
            }
            case PhysicsConfigWindow::HierarchicalGrid : {
                ftHierarchicalGrid* hGrid = new ftHierarchicalGrid();
                hGrid->setConfiguration(configWindow.hierarchicalConfig);
                broadphaseSystem = hGrid;
                break;
            }
            case PhysicsConfigWindow::ToroidalGrid : {
                ftToroidalGrid* tGrid = new ftToroidalGrid();
                tGrid->setConfiguration(configWindow.toroidalConfig);
                broadphaseSystem = tGrid;
                break;
            }
            case PhysicsConfigWindow::QuadTree : {
                ftQuadTree* quadTree = new ftQuadTree();
                quadTree->setConfiguration(configWindow.quadConfig);
                broadphaseSystem = quadTree;
                break;
            }
        }

        physicsSystem->installBroadphase(broadphaseSystem);
        physicsSystem->setConfiguration(configWindow.config);
        physicsSystem->init();
        FaltonDemo::demo[demoWindow.demo].Init(physicsSystem);
    }

    void HandleInput() {
        if (ImGui::IsKeyDown(GLFW_KEY_RIGHT)) {
            debugCamera.MoveRight();
        }
        if (ImGui::IsKeyDown(GLFW_KEY_LEFT)) {
            debugCamera.MoveLeft();
        }
        if (ImGui::IsKeyDown(GLFW_KEY_UP)) {
            debugCamera.MoveUp();
        }
        if (ImGui::IsKeyDown(GLFW_KEY_DOWN)) {
            debugCamera.MoveDown();
        }
        if (ImGui::IsKeyDown(GLFW_KEY_J)) {
            debugCamera.ZoomIn();
        }
        if (ImGui::IsKeyDown(GLFW_KEY_K)) {
            debugCamera.ZoomOut();
        }
    }

    void Update(ftPhysicsSystem* physicsSystem, ftWindowInfo windowInfo) {
 

        if (executionButton.starting) {
            InitAndLoadPhysics(physicsSystem);
        } else if (executionButton.restarting) {
            ftProfiler::Clear();
            physicsSystem->shutdown();
            delete broadphaseSystem;
            InitAndLoadPhysics(physicsSystem);
        }
        if (executionButton.stopping) {
            ftProfiler::Clear();
            physicsSystem->shutdown();
            delete broadphaseSystem;
        }
        if (executionButton.playing){
            ftProfiler::BeginFrame();
            physicsSystem->step(1.0/60);
            ftProfiler::EndFrame();
            performanceWindow.Refresh();
        }

        ftDebugCanvas *pCanvas = &canvas;

        pCanvas->SetCamera(debugCamera);
       
        if (executionButton.running) {   
            pCanvas->DrawPhysics(physicsSystem);
        }

        pCanvas->Render();

        demoWindow.Draw();
        executionButton.Draw();
        configWindow.Draw();
        performanceWindow.Draw();
            
    }

    void Cleanup() {
        canvas.Cleanup();
    }
}

#endif