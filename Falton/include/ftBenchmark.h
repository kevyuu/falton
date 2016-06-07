//
// Created by Kevin Yu on 2016-05-24.
//

#ifndef FALTON_FTBENCHMARK_H
#define FALTON_FTBENCHMARK_H

#include <string>
#include <vector>
#include <sys/time.h>
using namespace std;

//A benchmarking facility that only accomodate up to 100 tags
class ftBenchmark {

public:
    static int nTags;
    static int nFrame;

    struct BenchTable {
        string tag;
        vector<float> data;
    };

    struct BenchUnit {
        int index;
        double start;
    };

    static long getMicro() {
        timeval time;
        gettimeofday(&time, NULL);
        long micro = (time.tv_sec * 1000000) + (time.tv_usec);
        return micro;
    }

    static vector<BenchUnit> benchUnits;

    static BenchTable benchTables[100];
    static float averages[100];

    static void BeginFrame();
    static int Begin(string tag, int index);
    static void End();
    static void EndFrame();
    static void Clear();

};


#endif //FALTON_FTBENCHMARK_H
