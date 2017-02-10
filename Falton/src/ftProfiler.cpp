//
// Created by Kevin Yu on 2016-05-24.
//

#include "ftProfiler.h"

#include <iostream>

int ftProfiler::nTags = 0;
int ftProfiler::nFrame = 0;
long long ftProfiler::ReadTSCCycle = 0;
vector<ftProfiler::BenchUnit> ftProfiler::benchUnits;
ftProfiler::BenchTable ftProfiler::benchTables[100];

void ftProfiler::BeginFrame() {
    // ++nFrame;
    // for (int i = 0 ; i < nTags; ++i) {
    //     benchTables[i].data.push_back(0);
    // }
    // long long start = ReadTSC();
    // long long end = ReadTSC();
    // ReadTSCCycle = end - start;
}

void ftProfiler::EndFrame() {
    //do nothing
}

void ftProfiler::Clear() {
    // for (int i = 0 ; i < nTags; ++i) {
    //     benchTables[i].data.clear();
    // }
    // nFrame = 0;
}