//
// Created by Kevin Yu on 2016-05-19.
//

#ifndef FALTON_DEMOWINDOW_H
#define FALTON_DEMOWINDOW_H

#include <IMGUI/imgui.h>
#include <IMGUI/imgui_internal.h>
#include "../FaltonDemo.h"

struct DemoWindow {

    int demo;

    void Draw() {
        ImGui::SetNextWindowSize(ImVec2(200,100), ImGuiSetCond_FirstUseEver);
        ImGui::SetNextWindowPos(ImVec2(0,0),ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Demo");
        const char* demoTitles[FaltonDemo::NUM_DEMO];
        for (int i = 0; i < FaltonDemo::NUM_DEMO; ++i) {
            demoTitles[i] = FaltonDemo::demo[i].name;
        }
        ImGui::Combo("Demo", &demo, demoTitles, IM_ARRAYSIZE(demoTitles));
        ImGui::End();
    }
};

#endif //FALTON_DEMOWINDOW_H
