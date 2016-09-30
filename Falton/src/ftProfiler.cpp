//
// Created by Kevin Yu on 2016-05-24.
//

#include "ftBenchmark.h"

#include <iostream>

int ftBenchmark::nTags = 0;
int ftBenchmark::nFrame = 0;
long long ftBenchmark::ReadTSCCycle = 0;
vector<ftBenchmark::BenchUnit> ftBenchmark::benchUnits;
ftBenchmark::BenchTable ftBenchmark::benchTables[100];

void ftBenchmark::BeginFrame() {
    ++nFrame;
    for (int i = 0 ; i < nTags; ++i) {
        benchTables[i].data.push_back(0);
    }
    long long start = ReadTSC();
    long long end = ReadTSC();
    ReadTSCCycle = end - start;
}

void ftBenchmark::EndFrame() {
    //do nothing
}

void ftBenchmark::Clear() {
    for (int i = 0 ; i < nTags; ++i) {
        benchTables[i].data.clear();
    }
    nFrame = 0;
}