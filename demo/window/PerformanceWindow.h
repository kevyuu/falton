//
// Created by Kevin Yu on 2016-05-21.
//

#ifndef FALTON_PERFORMANCEWINDOW_H
#define FALTON_PERFORMANCEWINDOW_H

#include <IMGUI/imgui.h>

#include <iostream>
#include <fstream>
#include <streambuf>
#include <sstream>
#include <string>
#include <vector>
#include <ftProfiler.h>

struct PerformanceWindow {

    int startFrame;

    static constexpr int MIN_CYCLE = 0;
    static constexpr int MAX_CYCLE = 100000000;
    static constexpr int NUMBER_OF_SHOWN_FRAME = 100;
    static constexpr int MOV_AVERAGE_SIZE = 100;

    int currentFrame;

    std::vector<float> averages;

    void Refresh() {

        if (currentFrame != ftProfiler::nFrame) {
            int nFrame = ftProfiler::nFrame;

            if (ftProfiler::nFrame > NUMBER_OF_SHOWN_FRAME) startFrame = nFrame - NUMBER_OF_SHOWN_FRAME;
            else startFrame = 0;

            while (averages.size() < ftProfiler::nTags) {
                averages.push_back(0);
            }
            
            if (ftProfiler::nFrame > MOV_AVERAGE_SIZE) {
                for (int i = 0; i < ftProfiler::nTags; ++i) {
                    float prev100data = ftProfiler::benchTables[i].data[nFrame - MOV_AVERAGE_SIZE  - 1];
                    float newData = ftProfiler::benchTables[i].data[nFrame - 1];
                    averages[i] = averages[i] * MOV_AVERAGE_SIZE - prev100data + newData;
                    averages[i] /= MOV_AVERAGE_SIZE;
                }
            } else {
                for (int i = 0; i < ftProfiler::nTags; ++i) {
                    float newData = ftProfiler::benchTables[i].data[nFrame - 1];
                    averages[i] = averages[i] * (nFrame - 1) + newData;
                    averages[i] /= nFrame;
                }
            }

            currentFrame = ftProfiler::nFrame;
        }
    }

    void Draw() {

        ImGui::Begin("Performance Metric");

        int nFrame = ftProfiler::nFrame;

        for (int i = 0 ; i < ftProfiler::nTags; ++i) {
            char graphLabel[25] = "average = ";
            sprintf(graphLabel + 10, "%f", averages[i]);
            graphLabel[24] = '\0';
            ImGui::PlotLines(ftProfiler::benchTables[i].tag.c_str(),
                                &ftProfiler::benchTables[i].data[0] + startFrame,
                                NUMBER_OF_SHOWN_FRAME, 0,graphLabel, MIN_CYCLE, MAX_CYCLE, ImVec2(0, 80));
        }

        if (ftProfiler::nFrame < NUMBER_OF_SHOWN_FRAME) {
            ImGui::SliderInt("Start Frame", &startFrame, 0, 0);
        } else {
            ImGui::SliderInt("Start Frame", &startFrame, 0, nFrame - NUMBER_OF_SHOWN_FRAME);
        }

        ImGui::NewLine();
        ImGui::Separator();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvailWidth() * 0.3f);
        static char fileLocation[128] = "output\0"; ImGui::InputText("File Location", fileLocation, 64, ImGuiInputTextFlags_CharsNoBlank);
        static int numFrame = 1; ImGui::InputInt("Number of Frame", &numFrame);
        static int chartNum = 1; ImGui::InputInt("Chart number", &chartNum);
        if (ImGui::Button("Export")) {
            std::string filename(fileLocation);
            std::ofstream outputFile(filename);
            for (int i = 0; i < numFrame; ++i) {
                outputFile<< std::to_string(ftProfiler::benchTables[chartNum - 1].data[i]) <<endl;
            }
            outputFile.close();
        }

        ImGui::End();
    }

};
#endif //FALTON_PERFORMANCEWINDOW_H
