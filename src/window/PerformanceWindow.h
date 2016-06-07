//
// Created by Kevin Yu on 2016-05-21.
//

#ifndef FALTON_PERFORMANCEWINDOW_H
#define FALTON_PERFORMANCEWINDOW_H

#include "../imgui/imgui.h"

#include <iostream>
#include <vector>
#include <ftBenchmark.h>

using namespace std;
struct PerformanceWindow {

    int startFrame;

    struct BenchTable {
        string tag;
        vector<double> durations;
    };

    PerformanceWindow() {

    }

    void draw() {

        ImGui::Begin("Performance Metric");

        if (ftBenchmark::nFrame < 100) {
            for (int i = 0 ; i < ftBenchmark::nTags; ++i) {
                char graphLabel[25] = "average = ";
                sprintf(graphLabel + 10, "%f", ftBenchmark::averages[i]);
                graphLabel[24] = '\0';
                ImGui::PlotLines(ftBenchmark::benchTables[i].tag.c_str(),
                                 &ftBenchmark::benchTables[i].data[0],
                                 100, 0,graphLabel, 0, 10000, ImVec2(0, 80));
            }
            ImGui::SliderInt("Start Frame", &startFrame, 0, 0);

        } else {
            for (int i = 0 ; i < ftBenchmark::nTags; ++i) {
                char graphLabel[25] = "average = ";
                sprintf(graphLabel + 10, "%f", ftBenchmark::averages[i]);
                graphLabel[24] = '\0';
                ImGui::PlotLines(ftBenchmark::benchTables[i].tag.c_str(),
                                 &ftBenchmark::benchTables[i].data[0] + startFrame,
                                 100, 0,graphLabel, 0, 10000, ImVec2(0, 80));
            }
            ImGui::SliderInt("Start Frame", &startFrame, 0, ftBenchmark::nFrame - 100);
        }

        ImGui::End();
    }

    void updateStartFrame() {
        if (ftBenchmark::nFrame > 100) startFrame = ftBenchmark::nFrame - 100;
    }


};
#endif //FALTON_PERFORMANCEWINDOW_H
