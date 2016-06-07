//
// Created by Kevin Yu on 2016-05-24.
//

#include "ftBenchmark.h"

#include <iostream>
using namespace std;


int ftBenchmark::nTags = 0;
int ftBenchmark::nFrame = 0;
vector<ftBenchmark::BenchUnit> ftBenchmark::benchUnits;
ftBenchmark::BenchTable ftBenchmark::benchTables[100];
float ftBenchmark::averages[100] = {};

void ftBenchmark::BeginFrame() {
    ++nFrame;
    for (int i = 0 ; i < nTags; ++i) {
        benchTables[i].data.push_back(0);
    }
}

int ftBenchmark::Begin(string tag, int index) {

    if (index == -1) {
        index = nTags;
        benchTables[index].tag = tag;
        for (int i = 0 ; i < nFrame; ++i) {
            benchTables[index].data.push_back(0);
        }
        ++nTags;
    }

    BenchUnit benchUnit;
    benchUnit.index = index;
    benchUnit.start = getMicro();
    benchUnits.push_back(benchUnit);

    return index;

}

void ftBenchmark::End() {
    BenchUnit benchUnit = benchUnits.back();
    benchUnits.pop_back();
    float duration = getMicro() - benchUnit.start;
    benchTables[benchUnit.index].data[nFrame - 1] += duration;
}

void ftBenchmark::EndFrame() {
    for (int i = 0 ; i < nTags; ++i) {
        averages[i] = (averages[i] * (nFrame - 1) + benchTables[i].data[nFrame - 1]) / nFrame;
    }
}

void ftBenchmark::Clear() {
    for (int i = 0 ; i < nTags; ++i) {
        benchTables[i].data.clear();
    }
    for (int i = 0 ; i < nTags; ++i) {
        averages[i] = 0.0f;
    }
    nFrame = 0;
}