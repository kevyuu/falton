//
// Created by Kevin Yu on 2016-05-24.
//

#ifndef FALTON_FTBENCHMARK_H
#define FALTON_FTBENCHMARK_H

#include <string>
#include <vector>
// #include <intrin.h>

#define PROFILE_BEGIN(TAG) \
    do { \
        static int index = -1; \
        index = ftProfiler::Begin(TAG, index); \
    } while (0)

#define PROFILE_END() ftProfiler::End()

//A benchmarking facility that only accomodate up to 100 tags
class ftProfiler {

public:
    static int nTags;
    static int nFrame;
    static long long ReadTSCCycle;

    struct BenchTable {
        std::string tag;
        std::vector<float> data;
    };

    struct BenchUnit {
        int index;
        long long start;
    };
    
    // static long long ReadTSC() {      // Returns time stamp counter
    //     int dummy[4];           // For unused returns
    //     volatile int DontSkip;  // Volatile to prevent optimizing
    //     long long clock;        // Time
    //     __cpuid(dummy, 0);      // Serialize
    //     DontSkip = dummy[0];    // Prevent optimizing away cpuid
    //     clock = __rdtsc();      // Read time
    //     return clock;
    // }

    static std::vector<BenchUnit> benchUnits;
    
    static BenchTable benchTables[100];

    static void BeginFrame();

    static inline int Begin(std::string tag, int index) {
        // if (index == -1) {
        //     index = nTags;
        //     benchTables[index].tag = tag;
        //     for (int i = 0 ; i < nFrame; ++i) {
        //         benchTables[index].data.push_back(0);
        //     }
        //     ++nTags;
        // }

        // BenchUnit benchUnit;
        // benchUnit.index = index;
        // benchUnits.push_back(benchUnit);
        // benchUnits.back().start = ReadTSC();

        // return index;
        return index;
    }

    static inline void End() {
        // long long end = ReadTSC();
        // BenchUnit benchUnit = benchUnits.back();
        // benchUnits.pop_back();
        // float cycleElapsed = (float) (end - benchUnit.start - ReadTSCCycle);
        // benchTables[benchUnit.index].data[nFrame - 1] += cycleElapsed;
    }

    static void EndFrame();
    static void Clear();

};


#endif //FALTON_FTBENCHMARK_H
