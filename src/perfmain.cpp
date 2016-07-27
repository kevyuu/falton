//
// Created by Kevin Yu on 2016-07-14.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <ftBenchmark.h>

#include <falton/physics/collision/broadphase/ftDynamicBVH.h>
#include <falton/physics/collision/broadphase/ftToroidalGrid.h>
#include <falton/physics/collision/broadphase/ftQuadTree.h>
#include <falton/physics/collision/broadphase/ftHierarchicalGrid.h>
#include <falton/physics/dynamic/ftPhysicsSystem.h>
#include <falton/physics/ftMassComputer.h>

ftPhysicsSystem physicsSystem;
ftBroadphaseSystem* broadphaseSystem;

const int N_TEST_STEP = 1000;
const int N_TEST_SAMPLE = 10;

std::stringstream headBuffer;
std::stringstream tailBuffer;

enum BroadphaseType{
    DYNAMIC_BVH,
    TOROIDAL_GRID,
    HIERARCHICAL_GRID,
    QUAD_TREE
};

struct BroadphaseConfig {
    BroadphaseType type;
    std::string name;
    void* config;
};

typedef void (*TestInit) ();

void MeasurePerformance();
void MeasureTestCase1();
void MeasureTestCase2();
void MeasureTestCase3();
void MeasureTestCase4();
void Measure(ftPhysicsSystem::ftConfig* , std::string prefix,
             BroadphaseConfig* config, int nBroadphase, TestInit testInit);
void TestCase1_init();
void TestCase2_init();
void TestCase3_init();
void TestCase4_init();
void WriteArrayToFile(std::string filename, float* array, int nVal);
void WarmUp();
void CollectTimeAndMemoryData(TestInit testInit,float time1[], float time2[], uint64 memory[]);

int main(int argc __attribute__((unused)), char *argv[] __attribute__((unused))) {
    std::ifstream headFile("charts/headHTML");
    headBuffer << headFile.rdbuf();
    headFile.close();

    std::ifstream tailFile("charts/tailHTML");
    tailBuffer << tailFile.rdbuf();
    tailFile.close();
    WarmUp();
    MeasurePerformance();
}

void MeasurePerformance() {
//    MeasureTestCase1();

//    MeasureTestCase2();

//    MeasureTestCase3();

    MeasureTestCase4();
}

void WarmUp() {
    ftDynamicBVH::ftConfig bvhConfig;
    ftToroidalGrid::ftConfig toroidalConfig;
    ftHierarchicalGrid::ftConfig hashGridConfig;
    ftQuadTree::ftConfig quadConfig;

    ftPhysicsSystem::ftConfig physicsConfig;

    float time1[N_TEST_STEP], time2[N_TEST_STEP];
    uint64 memory[N_TEST_STEP];

    //bvh
    bvhConfig.aabbExtension = 0.05;
    ftDynamicBVH* dynamicBVH = new ftDynamicBVH();
    broadphaseSystem = dynamicBVH;
    dynamicBVH->setConfiguration(bvhConfig);
    physicsSystem.installBroadphase(dynamicBVH);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase1_init();
    for (int i = 0; i < 1000; ++i) {
        ftBenchmark::BeginFrame();
        physicsSystem.step(1.0/60);
        ftBenchmark::EndFrame();
    }
    ftBenchmark::Clear();
    physicsSystem.shutdown();

    toroidalConfig.cellSize = 1.5;
    toroidalConfig.nCol = 256;
    toroidalConfig.nRow = 32;
    ftToroidalGrid* toroidalGrid = new ftToroidalGrid();
    broadphaseSystem = toroidalGrid;
    toroidalGrid->setConfiguration(toroidalConfig);
    physicsSystem.installBroadphase(toroidalGrid);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase1_init();
    for (int i = 0; i < 1000; ++i) {
        ftBenchmark::BeginFrame();
        physicsSystem.step(1.0/60);
        ftBenchmark::EndFrame();
    }
    ftBenchmark::Clear();
    physicsSystem.shutdown();

    hashGridConfig.baseSize = 1.5;
    hashGridConfig.nBucket = 1024;
    hashGridConfig.nLevel = 16;
    hashGridConfig.sizeMul = 2;
    ftHierarchicalGrid* hierarchicalGrid = new ftHierarchicalGrid();
    broadphaseSystem = hierarchicalGrid;
    hierarchicalGrid->setConfiguration(hashGridConfig);
    physicsSystem.installBroadphase(hierarchicalGrid);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase1_init();
    for (int i = 0; i < 1000; ++i) {
        ftBenchmark::BeginFrame();
        physicsSystem.step(1.0/60);
        ftBenchmark::EndFrame();
    }
    ftBenchmark::Clear();
    physicsSystem.shutdown();

    quadConfig.maxLevel = 15;
    quadConfig.worldAABB.min = ftVector2(-55,-10);
    quadConfig.worldAABB.max = ftVector2(55,150);
    ftQuadTree* quadTree = new ftQuadTree();
    broadphaseSystem = quadTree;
    quadTree->setConfiguration(quadConfig);
    physicsSystem.installBroadphase(quadTree);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase1_init();
    for (int i = 0; i < 1000; ++i) {
        ftBenchmark::BeginFrame();
        physicsSystem.step(1.0/60);
        ftBenchmark::EndFrame();
    }
    ftBenchmark::Clear();
    physicsSystem.shutdown();
}

void MeasureTestCase1() {

    BroadphaseConfig config[4];

    ftDynamicBVH::ftConfig bvhConfig;
    bvhConfig.aabbExtension = 0.05;
    config[0].type = DYNAMIC_BVH;
    config[0].config = &bvhConfig;
    config[0].name = "BVH";

    ftToroidalGrid::ftConfig toroidalConfig;
    toroidalConfig.cellSize = 1.5;
    toroidalConfig.nCol = 256;
    toroidalConfig.nRow = 32;
    config[1].type = TOROIDAL_GRID;
    config[1].config = &toroidalConfig;
    config[1].name = "Toroidal";

    ftHierarchicalGrid::ftConfig hashGridConfig;
    hashGridConfig.baseSize = 1.5;
    hashGridConfig.nBucket = 2048;
    hashGridConfig.nLevel = 16;
    hashGridConfig.sizeMul = 2;
    config[2].type = HIERARCHICAL_GRID;
    config[2].config = &hashGridConfig;
    config[2].name = "HierarchicalGrid";

    ftQuadTree::ftConfig quadConfig;
    quadConfig.maxLevel = 15;
    quadConfig.worldAABB.min = ftVector2(-55,-10);
    quadConfig.worldAABB.max = ftVector2(55,150);
    config[3].type = QUAD_TREE;
    config[3].config = &quadConfig;
    config[3].name = "QuadTree";

    ftPhysicsSystem::ftConfig physicsConfig;
    physicsConfig.sleepRatio = 0;
    Measure(&physicsConfig, "t1_all", config, 4, TestCase1_init);

    physicsConfig.sleepRatio = 1;
    Measure(&physicsConfig, "t1_one", config, 4, TestCase1_init);

}

void MeasureTestCase2() {

    BroadphaseConfig config[4];

    ftDynamicBVH::ftConfig bvhConfig;
    bvhConfig.aabbExtension = 0.05;
    config[0].type = DYNAMIC_BVH;
    config[0].config = &bvhConfig;
    config[0].name = "BVH";

    ftToroidalGrid::ftConfig toroidalConfig;
    toroidalConfig.cellSize = 1.5;
    toroidalConfig.nCol = 64;
    toroidalConfig.nRow = 64;
    config[1].type = TOROIDAL_GRID;
    config[1].config = &toroidalConfig;
    config[1].name = "Toroidal";

    ftHierarchicalGrid::ftConfig hashGridConfig;
    hashGridConfig.baseSize = 1.5;
    hashGridConfig.nBucket = 4096;
    hashGridConfig.nLevel = 16;
    hashGridConfig.sizeMul = 2;
    config[2].type = HIERARCHICAL_GRID;
    config[2].config = &hashGridConfig;
    config[2].name = "HierarchicalGrid";

    ftQuadTree::ftConfig quadConfig;
    quadConfig.maxLevel = 15;
    quadConfig.worldAABB.min = ftVector2(-10,-10);
    quadConfig.worldAABB.max = ftVector2(36000,36000);
    config[3].type = QUAD_TREE;
    config[3].config = &quadConfig;
    config[3].name = "QuadTree";

    ftPhysicsSystem::ftConfig physicsConfig;
    physicsConfig.sleepRatio = 0;
    Measure(&physicsConfig, "t2_all", config, 4, TestCase2_init);

    physicsConfig.sleepRatio = 1;
    Measure(&physicsConfig, "t2_one", config, 4, TestCase2_init);

}

void MeasureTestCase3() {
    BroadphaseConfig config[4];

    ftDynamicBVH::ftConfig bvhConfig;
    bvhConfig.aabbExtension = 0.05;
    config[0].type = DYNAMIC_BVH;
    config[0].config = &bvhConfig;
    config[0].name = "BVH";

    ftToroidalGrid::ftConfig toroidalConfig;
    toroidalConfig.cellSize = 1.5;
    toroidalConfig.nCol = 32;
    toroidalConfig.nRow = 32;
    config[1].type = TOROIDAL_GRID;
    config[1].config = &toroidalConfig;
    config[1].name = "Toroidal";

    ftHierarchicalGrid::ftConfig hashGridConfig;
    hashGridConfig.baseSize = 1.5;
    hashGridConfig.nBucket = 512;
    hashGridConfig.nLevel = 16;
    hashGridConfig.sizeMul = 2;
    config[2].type = HIERARCHICAL_GRID;
    config[2].config = &hashGridConfig;
    config[2].name = "HierarchicalGrid";

    ftQuadTree::ftConfig quadConfig;
    quadConfig.maxLevel = 15;
    quadConfig.worldAABB.min = ftVector2(-150,-10);
    quadConfig.worldAABB.max = ftVector2(150,350);
    config[3].type = QUAD_TREE;
    config[3].config = &quadConfig;
    config[3].name = "QuadTree";

    ftPhysicsSystem::ftConfig physicsConfig;

    physicsConfig.sleepRatio = 0;
    Measure(&physicsConfig, "t3_all", config, 4, TestCase3_init);

    physicsConfig.sleepRatio = 1;
    Measure(&physicsConfig, "t3_one", config, 4, TestCase3_init);
}

void MeasureTestCase4() {

    BroadphaseConfig config[4];

    ftDynamicBVH::ftConfig bvhConfig;
    bvhConfig.aabbExtension = 0.05;
    config[0].type = DYNAMIC_BVH;
    config[0].config = &bvhConfig;
    config[0].name = "BVH";

    ftToroidalGrid::ftConfig toroidalConfig;
    toroidalConfig.cellSize = 20;
    toroidalConfig.nCol = 32;
    toroidalConfig.nRow = 4;
    config[1].type = TOROIDAL_GRID;
    config[1].config = &toroidalConfig;
    config[1].name = "Toroidal";

    ftHierarchicalGrid::ftConfig hashGridConfig;
    hashGridConfig.baseSize = 1.5;
    hashGridConfig.nBucket = 256;
    hashGridConfig.nLevel = 16;
    hashGridConfig.sizeMul = 2;
    config[2].type = HIERARCHICAL_GRID;
    config[2].config = &hashGridConfig;
    config[2].name = "HierarchicalGrid";

    ftQuadTree::ftConfig quadConfig;
    quadConfig.maxLevel = 15;
    quadConfig.worldAABB.min = ftVector2(-10,-10);
    quadConfig.worldAABB.max = ftVector2(5000,200);
    config[3].type = QUAD_TREE;
    config[3].config = &quadConfig;
    config[3].name = "QuadTree";

    ftPhysicsSystem::ftConfig physicsConfig;

    physicsConfig.sleepRatio = 0;
    Measure(&physicsConfig, "t4_all", config, 4, TestCase4_init);

    physicsConfig.sleepRatio = 1;
    Measure(&physicsConfig, "t4_one", config, 4, TestCase4_init);


}

void Measure(ftPhysicsSystem::ftConfig* physicsConfig, std::string prefix,
             BroadphaseConfig* broadphaseConfig, int nBroadphase, TestInit testInit) {

    float time1[N_TEST_STEP], time2[N_TEST_STEP];
    float memoryFloat[N_TEST_STEP];
    uint64 memory[N_TEST_STEP];

    string filename = "";

    for (int i = 0; i < nBroadphase; ++i) {
        filename = broadphaseConfig[i].name;
        if (broadphaseConfig[i].type == DYNAMIC_BVH) {
            ftDynamicBVH* dynamicBVH = new ftDynamicBVH();
            broadphaseSystem = dynamicBVH;
            dynamicBVH->setConfiguration(*(ftDynamicBVH::ftConfig*)broadphaseConfig[i].config);
        } else if (broadphaseConfig[i].type == TOROIDAL_GRID) {
            ftToroidalGrid* toroidalGrid = new ftToroidalGrid();
            broadphaseSystem = toroidalGrid;
            toroidalGrid->setConfiguration(*(ftToroidalGrid::ftConfig*)broadphaseConfig[i].config);
        } else if (broadphaseConfig[i].type == HIERARCHICAL_GRID) {
            ftHierarchicalGrid* hierarchicalGrid = new ftHierarchicalGrid();
            broadphaseSystem = hierarchicalGrid;
            hierarchicalGrid->setConfiguration(*(ftHierarchicalGrid::ftConfig*)broadphaseConfig[i].config);
        } else if (broadphaseConfig[i].type == QUAD_TREE) {
            ftQuadTree* quadTree = new ftQuadTree();
            broadphaseSystem = quadTree;
            quadTree->setConfiguration(*(ftQuadTree::ftConfig*)broadphaseConfig[i].config);
        }
        physicsSystem.installBroadphase(broadphaseSystem);
        physicsSystem.setConfiguration(*physicsConfig);
        CollectTimeAndMemoryData(testInit,time1, time2, memory);
        WriteArrayToFile("chartInput/"+prefix+"_"+filename+"_time1", time1, N_TEST_STEP);
        WriteArrayToFile("chartInput/"+prefix+"_"+filename+"_time2", time2, N_TEST_STEP);
        for (int j = 0 ; j < N_TEST_STEP; ++j) {
            memoryFloat[j] = (float) memory[j];
        }
        WriteArrayToFile("chartInput/"+prefix+"_"+filename+"_memory", memoryFloat, N_TEST_STEP);

        delete broadphaseSystem;
    }
}

void CollectTimeAndMemoryData(TestInit testInit,float time1[], float time2[], uint64 memory[]) {

    physicsSystem.init();
    testInit();
    cout<<"Iteration : "<<1<<std::endl;
    for (int i = 0; i < N_TEST_STEP; ++i) {
        ftBenchmark::BeginFrame();
        physicsSystem.step(1.0/60);
        ftBenchmark::EndFrame();
        time1[i] = ftBenchmark::benchTables[0].data[i];
        time2[i] = ftBenchmark::benchTables[1].data[i];
        memory[i] = broadphaseSystem->getMemoryUsage();
    }
    ftBenchmark::Clear();
    physicsSystem.shutdown();


    for (int iter = 1; iter < N_TEST_SAMPLE; ++iter) {
        cout<<"Iteration : "<<iter + 1<<std::endl;
        physicsSystem.init();
        testInit();
        for (int i = 0; i < N_TEST_STEP; ++i) {
            ftBenchmark::BeginFrame();
            physicsSystem.step(1.0/60);
            ftBenchmark::EndFrame();
            time1[i] += ftBenchmark::benchTables[0].data[i];
            time2[i] += ftBenchmark::benchTables[1].data[i];
        }
        ftBenchmark::Clear();
        physicsSystem.shutdown();
    }

    for (int i = 0; i < N_TEST_STEP; ++i) {
        time1[i] /= N_TEST_SAMPLE;
        time2[i] /= N_TEST_SAMPLE;
    }

}

void WriteArrayToFile(std::string filename, float* array, int nVal) {
    std::ofstream outputFile(filename);
    for (int i = 0 ; i < nVal ; ++i) {
        outputFile << array[i] << endl;
    }
    outputFile.close();
}

ftBody* createDynamicBox(ftVector2 position, ftVector2 halfWidth, real mass, real friction) {
    ftPolygon *boxShape = ftPolygon::createBox(-1 * halfWidth, halfWidth);
    ftMassProperty boxmp = ftMassComputer::computeForPolygon(*boxShape,mass, ftVector2(0,0));
    ftBody* box = physicsSystem.createDynamicBody(position,0, boxmp.mass, boxmp.moment);
    box->centerOfMass = boxmp.centerOfMass;
    box->moment = boxmp.moment;

    ftCollider* boxCollider = physicsSystem.createCollider(box, boxShape, ftVector2(0,0), 0);
    boxCollider->friction = friction;
    boxCollider->restitution = 0;
    delete boxShape;

    return box;
}


ftBody* createBall(ftVector2 position, real mass, real radius, real friction, real restitution) {
    ftCircle *ballShape = ftCircle::create(radius);
    ftMassProperty ballmp = ftMassComputer::computeForCircle(*ballShape,mass, ftVector2(0,0));
    ftBody* ball = physicsSystem.createDynamicBody(position, M_PI/8, ballmp.mass, ballmp.moment);
    ball->centerOfMass = ballmp.centerOfMass;

    ftCollider* ballCollider = physicsSystem.createCollider(ball, ballShape, ftVector2(0,0), 0);
    ballCollider->friction = friction;
    ballCollider->restitution = restitution;

    delete ballShape;

    return ball;
}

ftBody*  createStaticBox(ftVector2 position, real orientation, ftVector2 halfWidth, real friction) {
    ftPolygon *groundShape = ftPolygon::createBox(-1 * halfWidth, halfWidth);
    ftBody* ground = physicsSystem.createStaticBody(position,orientation);

    ftCollider* collider = physicsSystem.createCollider(ground, groundShape, ftVector2(0,0), 0);
    collider->friction = friction;

    delete groundShape;

    return ground;
}


//test liquid
void TestCase1_init() {
    createStaticBox(ftVector2(0,0), 0,ftVector2(50,0.5), 0.6);
    createStaticBox(ftVector2(-50,50), 0,ftVector2(1,50), 0.2);
    createStaticBox(ftVector2(50,50), 0,ftVector2(1,50), 0.2);

    ftVector2 x(-30, 0.75);
    ftVector2 y;

    for (int i = 0; i < 60 ; ++i) {
        y = x;
        for (int j = 0; j < 30; ++j) {
            ftBody* body = createBall(y, 1, 0.5, 0, 0.6);
            body->userdata = (void*)(i*j);
            y += ftVector2(1.125f, 0.0f);
        }
        x += ftVector2(0.5625f, 2);
    }
}

//test sparse pyramid
void TestCase2_init() {
    int32 spacing = 4000;
    for (int k = 0 ; k < 8; ++k) {
        for (int l = 0 ; l < 8; ++l) {
            createStaticBox(ftVector2(-3 + spacing * l, spacing * k), 0,ftVector2(6,0.1), 0.2);
            ftVector2 x(-6 + spacing * l, 0.75 + spacing * k);
            ftVector2 y;

            for (int i = 0; i < 7; ++i) {
                y = x;
                for (int j = i; j < 7; ++j) {
                    createDynamicBox(y, ftVector2(0.5, 0.5), 10, 0.2);
                    y += ftVector2(1.125f, 0.0f);
                }
                x += ftVector2(0.5625f, 2.0f);
            }

        }
    }
}

//obj different size
void TestCase3_init() {
    createStaticBox(ftVector2(0,0), 0,ftVector2(120,0.5), 0.6);
    createStaticBox(ftVector2(-122,50), 0,ftVector2(2,50), 0.2);
    createStaticBox(ftVector2(122,50), 0,ftVector2(2,50), 0.2);

    ftVector2 x(-30, 0.75);
    ftVector2 y;

    real density = 1;

    createDynamicBox(ftVector2(0,250), ftVector2(50, 50), 10000 * density,  0.2);
    createDynamicBox(ftVector2(-100,250), ftVector2(20, 20),1600 * density,  0.2);

    std::srand(0);
    for (int i = 0; i < 30 ; ++i) {
        float maxWidth = 0.0f;
        y = x;
        for (int j = i; j < 30; ++j) {
            int random = std::rand() % 2;
            float width = 5;
            if (random == 0) {
                width = 2;
            }
            float halfWidth = width/2;
            y += ftVector2(halfWidth, 0.0f);
            createDynamicBox(y, ftVector2(halfWidth, halfWidth), width * width * density, 0.4);
            y += ftVector2(halfWidth, 0.0f);
            if (maxWidth < width) maxWidth = width;
        }
        x += ftVector2(0.5625, maxWidth + 0.25);
    }
}

void createPendulum(ftVector2 position) {
    ftBody* ceiling = createStaticBox(position, 0, ftVector2(0.5,0.5), 0.6);
    ftBody* ball = createBall(ftVector2(position.x - 5, position.y), 1, 0.5, 0.2, 0.8);
    physicsSystem.createDistanceJoint(ceiling, ball, ftVector2(0,0), ftVector2(0,0));
}

//platformer
void TestCase4_init() {
    int nPendulum = 6;
    real spacing = 0.001;
    int leftPendulum = nPendulum / 2;
    ftVector2 position(- leftPendulum * (1.0 + spacing * 2) + 25, 15);
    createPendulum(position);

    for (int i = 1; i < nPendulum; ++i) {
        position += ftVector2(1.0 + spacing, 0);
        ftBody *ceiling = createStaticBox(position, 0, ftVector2(0.5, 0.5), 0.6);
        ftBody *ball = createBall(ftVector2(position.x, position.y - 5), 1, 0.5, 0.2, 0.88);
        physicsSystem.createDistanceJoint(ceiling, ball, ftVector2(0, 0), ftVector2(0, 0.4));
    }

    for (int i = 0; i < 30; ++i) {
        real xOffset = i * 160;
        createStaticBox(ftVector2(xOffset + 25, 4), 0, ftVector2(25, 4), 0.6);
        createStaticBox(ftVector2(xOffset + 60, 10), 0, ftVector2(10, 10), 0.6);
        createStaticBox(ftVector2(xOffset + 80, 20), 0, ftVector2(10, 20), 0.6);
        createStaticBox(ftVector2(xOffset + 100, 30), 0, ftVector2(10, 30), 0.6);
        createStaticBox(ftVector2(xOffset + 140, 56), 0, ftVector2(10, 4), 0.6);
    }
}
