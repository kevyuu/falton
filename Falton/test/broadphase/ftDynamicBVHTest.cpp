//
// Created by Kevin Yu on 2016-05-18.
//

#include "gtest/gtest.h"
#include "falton/physics/collision/broadphase/ftNSquaredBroadphase.h"
#include "falton/physics/collision/broadphase/ftDynamicBVH.h"
#include <iostream>
#include <fstream>
#include "falton/container/ftChunkArray.h"

using namespace std;

class BVHTestFixture: public ::testing::Test {


};
TEST(BVHTest, testcase1) {
    ftChunkArray<ftCollisionShape> shape;
    shape.init(64);

    ifstream testcaseFile;
    testcaseFile.open("testcase/testcase1");
    string output;
    while (!testcaseFile.eof()) {
        testcaseFile >> output;
        stringstream ss(output);

        string vertex1x, vertex1y;
        string vertex2x, vertex2y;
        string vertex3x, vertex3y;
        string vertex4x, vertex4y;

        ss >> vertex1x >> vertex1y;
        ss >> vertex2x >> vertex2y;
        ss >> vertex3x >> vertex3y;
        ss >> vertex4x >> vertex4y;
    }


    ftNSquaredBroadphase* broadphaseDummy = new ftNSquaredBroadphase();
    ftDynamicBVH* dynamicBVH = new ftDynamicBVH();

    broadphaseDummy->init();
    dynamicBVH->init();



    broadphaseDummy->shutdown();
    dynamicBVH->shutdown();

    shape.cleanup();
}