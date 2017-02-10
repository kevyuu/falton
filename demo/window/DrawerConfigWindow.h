//
// Created by Kevin Yu on 2016-05-22.
//

#ifndef FALTON_DRAWERCONFIGWINDOW_H
#define FALTON_DRAWERCONFIGWINDOW_H

#include "../imgui/imgui.h"
#include "../DrawPhysics.h"

struct DrawerConfigWindow {

    DrawConfig config;

    void draw() {
        ImGui::SetNextWindowPos(ImVec2(0,0),ImGuiSetCond_Once);
        ImGui::Begin("Draw Configuration");

        ImGui::Text("Static Body");
        ImGui::Checkbox("visible##static", &config.showStatic);
        ImGui::ColorEdit3("border##static", (float*)&config.staticBorder);
        ImGui::Separator();

        ImGui::Text("Kinematic Body");
        ImGui::Checkbox("visible##kinematic", &config.showKinematic);
        ImGui::ColorEdit3("border##kinematic", (float*)&config.kinematicBorder);
        ImGui::Separator();

        ImGui::Text("Dynamic Body");
        ImGui::Checkbox("visible##dynamic", &config.showDynamic);
        ImGui::ColorEdit3("border##dynamic", (float*)&config.dynamicBorder);
        ImGui::Separator();

        ImGui::Text("Sleeping Body");
        ImGui::Checkbox("visible##sleeping", &config.showSleeping);
        ImGui::ColorEdit3("border##sleeping", (float*)&config.sleepingBorder);
        ImGui::Separator();

        ImGui::End();

    }
};
#endif //FALTON_DRAWERCONFIGWINDOW_H
