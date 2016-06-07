//
// Created by Kevin Yu on 2016-05-19.
//

#ifndef FALTON_PLAYPAUSEBUTTON_H
#define FALTON_PLAYPAUSEBUTTON_H

#include "../imgui/imgui.h"

struct ExecutionButton {

    bool starting = false;
    bool restarting = false;
    bool playing = false;
    bool running = false;
    bool stopping = false;

    const char* runActive = "Restart";
    const char* runDeactive = "Start";
    const char* playActive = "Pause";
    const char* playDeactive = "Play";
    const char* stopText = "Stop";

    const ImVec2 buttonSize = ImVec2(50.0 , 15.0);

    const ImColor buttonDisabledColor = ImColor(200,200,200);
    const ImColor buttonActiveColor = ImColor(200, 10, 10);

    void draw() {

        ImGuiWindowFlags window_flags = 0;
        window_flags |= ImGuiWindowFlags_NoTitleBar;
        window_flags |= ImGuiWindowFlags_NoResize;
        window_flags |= ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoScrollbar;
        window_flags |= ImGuiWindowFlags_NoCollapse;
        ImGui::SetNextWindowPosCenter(ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Play", nullptr, window_flags);

        starting = false;
        restarting = false;
        stopping = false;

        const char* runText = runDeactive;
        if (running) {
            runText = runActive;
        }

        const char* playText = playDeactive;
        if (playing) playText = playActive;

        if (ImGui::Button(runText, buttonSize)) {
            if (running) restarting = true;
            else starting = true;
            running = true;
            playing = true;
        }


        bool changeColor = false;
        ImGui::PushID(1);
        if (!running) {
            changeColor = true;
            ImGui::PushStyleColor(ImGuiCol_Button, buttonDisabledColor);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, buttonDisabledColor);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, buttonDisabledColor);
        }
        ImGui::SameLine();
        if (ImGui::Button(playText, buttonSize)) {
            if (running) playing = !playing;
        }

        ImGui::SameLine();
        if (ImGui::Button(stopText, buttonSize)) {
            stopping = true;
            running = false;
            playing = false;
        }

        if (changeColor) ImGui::PopStyleColor(3);
        ImGui::PopID();


        ImGui::End();
    }

};

#endif //FALTON_PLAYPAUSEBUTTON_H
