//
// Created by Kevin Yu on 2016-05-19.
//

#ifndef FALTON_DEMOWINDOW_H
#define FALTON_DEMOWINDOW_H

#include "../imgui/imgui.h"
#include "../imgui/imgui_internal.h"

struct DemoWindow {

    int demo;

    void draw() {
        ImGui::SetNextWindowSize(ImVec2(200,100), ImGuiSetCond_FirstUseEver);
        ImGui::SetNextWindowPos(ImVec2(0,0),ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Demo");
        const char* demoTitles[] = {"Basic", "Friction", "Stacking", "Pyramid", "Tuttee", "Restitution", "Sparse Pyramid", "Motor", "Distance Joint", "Spring", "Liquid", "Dominoes", "Demo13", "Platformer"};
        ImGui::Combo("Demo", &demo, demoTitles, IM_ARRAYSIZE(demoTitles));
        ImGui::End();
    }
};

#endif //FALTON_DEMOWINDOW_H
