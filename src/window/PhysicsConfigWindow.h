//
// Created by Kevin Yu on 2016-05-20.
//

#ifndef FALTON_PHYSICSWINDOW_H
#define FALTON_PHYSICSWINDOW_H

#include <falton/physics/collision/broadphase/ftHierarchicalGrid.h>
#include <falton/physics/collision/broadphase/ftDynamicBVH.h>
#include <falton/physics/collision/broadphase/ftToroidalGrid.h>
#include <falton/physics/collision/broadphase/ftQuadTree.h>
#include "../imgui/imgui.h"
#include "../imgui/imgui_internal.h"

struct PhysicsConfigWindow {

    ftPhysicsSystem::ftConfig config;
    ftHierarchicalGrid::ftConfig hierarchicalConfig;
    ftDynamicBVH::ftConfig bvhConfig;
    ftToroidalGrid::ftConfig toroidalConfig;
    ftQuadTree::ftConfig quadConfig;

    enum Broadphase {
        NSquaredBroadphase,
        HierarchicalGrid,
        DynamicBVH,
        ToroidalGrid,
        QuadTree
    } broadphase;

    const char* windowTitle = "Physics Configuration";
    const char* broadphaseNames[5] = {"NSquared Broadphase", "Hierarchical Grid", "Dynamic BVH", "ToroidalGrid", "QuadTree"};

    void draw() {

        ImGui::SetNextWindowSize(ImVec2(450,300), ImGuiSetCond_Once);
        ImGui::Begin(windowTitle);

        ImGui::PushItemWidth(ImGui::GetContentRegionAvailWidth() * 0.5f);
        ImGui::Text("General");
        ImGui::InputFloat("Time to sleep", &config.sleepTimeLimit);
        ImGui::InputFloat("Linear sleep limit", &config.sleepLinearLimit);
        ImGui::InputFloat("Angular sleep limit", &config.sleepAngularLimit);
        ImGui::SliderFloat("Sleep Ratio", &config.sleepRatio, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Baumgarte Coefficient", &config.solverConfig.baumgarteCoef, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Allowed Penetration", &config.solverConfig.allowedPenetration, 0.0f, 1.0f, "%.3f");
        ImGui::InputInt("Solver Iteration",(int*)&config.solverConfig.numIteration);
        ImGui::InputFloat2("Gravity",(float*)&config.gravity);
        ImGui::NewLine();
        ImGui::Separator();
        ImGui::Text("Broadphase");
        ImGui::Combo("Broadphase", reinterpret_cast<int*>(&broadphase), broadphaseNames, IM_ARRAYSIZE(broadphaseNames));

        switch (broadphase) {
            case HierarchicalGrid : {
                ImGui::InputInt("Number Of Level", (int *)&hierarchicalConfig.nLevel);
                ImGui::InputFloat("Base Size", &hierarchicalConfig.baseSize);
                ImGui::InputFloat("Size Multiplier",&hierarchicalConfig.sizeMul);
                ImGui::InputInt("Number Of Bucket", (int*)&hierarchicalConfig.nBucket);
                break;
            }
            case DynamicBVH : {
                ImGui::InputFloat("AABB Extension", &bvhConfig.aabbExtension);
                break;
            }
            case ToroidalGrid : {
                ImGui::InputFloat("Cell Size", &toroidalConfig.cellSize);
                ImGui::InputInt("nRow", (int*)&toroidalConfig.nRow);
                ImGui::InputInt("nColumn", (int*)&toroidalConfig.nCol);
                break;
            }
            case QuadTree : {
                ImGui::InputFloat2("min", (float*)&quadConfig.worldAABB.min);
                ImGui::InputFloat2("max", (float*)&quadConfig.worldAABB.max);
                ImGui::InputInt("Max Level", (int*)&quadConfig.maxLevel);
                break;
            }
            default : {
                break;
            }
        }

        ImGui::PopItemWidth();
        ImGui::End();
    }
};

#endif //FALTON_PHYSICSWINDOW_H
