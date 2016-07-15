//
// Created by Kevin Yu on 2016-07-14.
//

#include <iostream>
#include <fstream>

#include <falton/physics/collision/broadphase/ftDynamicBVH.h>
#include <falton/physics/collision/broadphase/ftToroidalGrid.h>
#include <falton/physics/collision/broadphase/ftQuadTree.h>
#include <falton/physics/collision/broadphase/ftHierarchicalGrid.h>
#include <falton/physics/dynamic/ftPhysicsSystem.h>
#include <ftBenchmark.h>
#include <falton/physics/ftMassComputer.h>

void MeasurePerformance();
void MeasureTestCase1();
void MeasureTestCase2();
void MeasureTestCase3();
void MeasureTestCase4();
void TestCase1_init();
void TestCase2_init();
void TestCase3_init();
void TestCase4_init();
void CollectTimeAndMemoryData(std::string filename);

ftPhysicsSystem physicsSystem;
ftBroadphaseSystem* broadphaseSystem;

const int N_TEST_STEP = 1000;
const int N_TEST_SAMPLE = 100;

int main(int argc __attribute__((unused)), char *argv[] __attribute__((unused))) {
    MeasurePerformance();
}

void MeasurePerformance() {
    ftDynamicBVH::ftConfig bvhConfig;
    ftToroidalGrid::ftConfig toroidalConfig;
    ftHierarchicalGrid::ftConfig hashGridConfig;
    ftQuadTree::ftConfig quadConfig;

    ftPhysicsSystem::ftConfig physicsConfig;

    bvhConfig.aabbExtension = 0.05;
    ftDynamicBVH* dynamicBVH = new ftDynamicBVH();
    dynamicBVH->setConfiguration(bvhConfig);
    physicsSystem.installBroadphase(dynamicBVH);
    physicsSystem.init();

    TestCase1_init();
    MeasureTestCase1();

    TestCase2_init();
    MeasureTestCase2();

    TestCase3_init();
    MeasureTestCase3();

    TestCase4_init();
    MeasureTestCase4();
}

void MeasureTestCase1() {
    ftDynamicBVH::ftConfig bvhConfig;
    ftToroidalGrid::ftConfig toroidalConfig;
    ftHierarchicalGrid::ftConfig hashGridConfig;
    ftQuadTree::ftConfig quadConfig;

    ftPhysicsSystem::ftConfig physicsConfig;

    cout << "Test Case : 1"<<std::endl;
    cout<<"Algorithm : BVH"<<std::endl;
    //bvh
    bvhConfig.aabbExtension = 0.05;
    ftDynamicBVH* dynamicBVH = new ftDynamicBVH();
    broadphaseSystem = dynamicBVH;
    dynamicBVH->setConfiguration(bvhConfig);
    physicsSystem.installBroadphase(dynamicBVH);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase1_init();
    CollectTimeAndMemoryData("bvh1");
    physicsSystem.shutdown();

    cout<<"Algorithm : Toroidal Grid"<<std::endl;
    //toroidal
    toroidalConfig.cellSize = 1.5;
    toroidalConfig.nCol = 32;
    toroidalConfig.nRow = 64;
    ftToroidalGrid* toroidalGrid = new ftToroidalGrid();
    broadphaseSystem = toroidalGrid;
    toroidalGrid->setConfiguration(toroidalConfig);
    physicsSystem.installBroadphase(toroidalGrid);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase1_init();
    CollectTimeAndMemoryData("toroidal1");
    physicsSystem.shutdown();

    cout<<"Algorithm : Hash Grid"<<std::endl;
    //hashgrid
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
    CollectTimeAndMemoryData("hierarchical1");
    physicsSystem.shutdown();

    cout<<"Algorithm : Quad Tree"<<std::endl;
    //quad
    quadConfig.maxLevel = 16;
    quadConfig.worldAABB.min = ftVector2(-55,-10);
    quadConfig.worldAABB.max = ftVector2(55,110);
    ftQuadTree* quadTree = new ftQuadTree();
    broadphaseSystem = quadTree;
    quadTree->setConfiguration(quadConfig);
    physicsSystem.installBroadphase(quadTree);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase1_init();
    CollectTimeAndMemoryData("Quad1");
    physicsSystem.shutdown();

}

void MeasureTestCase2() {
    ftDynamicBVH::ftConfig bvhConfig;
    ftToroidalGrid::ftConfig toroidalConfig;
    ftHierarchicalGrid::ftConfig hashGridConfig;
    ftQuadTree::ftConfig quadConfig;

    ftPhysicsSystem::ftConfig physicsConfig;

    //bvh
    bvhConfig.aabbExtension = 0.05;
    ftDynamicBVH* dynamicBVH = new ftDynamicBVH();
    broadphaseSystem = dynamicBVH;
    dynamicBVH->setConfiguration(bvhConfig);
    physicsSystem.installBroadphase(dynamicBVH);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase2_init();
    CollectTimeAndMemoryData("bvh2");
    physicsSystem.shutdown();

    //toroidal
    toroidalConfig.cellSize = 1.5;
    toroidalConfig.nCol = 64;
    toroidalConfig.nRow = 64;
    ftToroidalGrid* toroidalGrid = new ftToroidalGrid();
    broadphaseSystem = toroidalGrid;
    toroidalGrid->setConfiguration(toroidalConfig);
    physicsSystem.installBroadphase(toroidalGrid);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase2_init();
    CollectTimeAndMemoryData("toroidal2");
    physicsSystem.shutdown();


    //hashgrid
    hashGridConfig.baseSize = 1.5;
    hashGridConfig.nBucket = 1024;
    hashGridConfig.nLevel = 3;
    hashGridConfig.sizeMul = 8;
    ftHierarchicalGrid* hierarchicalGrid = new ftHierarchicalGrid();
    broadphaseSystem = hierarchicalGrid;
    hierarchicalGrid->setConfiguration(hashGridConfig);
    physicsSystem.installBroadphase(hierarchicalGrid);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase2_init();
    CollectTimeAndMemoryData("hierarchical2");
    physicsSystem.shutdown();

    //quad
    quadConfig.maxLevel = 16;
    quadConfig.worldAABB.min = ftVector2(-50,-50);
    quadConfig.worldAABB.max = ftVector2(4050,4050);
    ftQuadTree* quadTree = new ftQuadTree();
    broadphaseSystem = quadTree;
    quadTree->setConfiguration(quadConfig);
    physicsSystem.installBroadphase(quadTree);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase2_init();
    CollectTimeAndMemoryData("Quad2");
    physicsSystem.shutdown();
}

void MeasureTestCase3() {
    ftDynamicBVH::ftConfig bvhConfig;
    ftToroidalGrid::ftConfig toroidalConfig;
    ftHierarchicalGrid::ftConfig hashGridConfig;
    ftQuadTree::ftConfig quadConfig;

    ftPhysicsSystem::ftConfig physicsConfig;

    //bvh
    bvhConfig.aabbExtension = 0.05;
    ftDynamicBVH* dynamicBVH = new ftDynamicBVH();
    broadphaseSystem = dynamicBVH;
    dynamicBVH->setConfiguration(bvhConfig);
    physicsSystem.installBroadphase(dynamicBVH);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase3_init();
    CollectTimeAndMemoryData("bvh3");
    physicsSystem.shutdown();

    //toroidal
    toroidalConfig.cellSize = 1.5;
    toroidalConfig.nCol = 64;
    toroidalConfig.nRow = 64;
    ftToroidalGrid* toroidalGrid = new ftToroidalGrid();
    broadphaseSystem = toroidalGrid;
    toroidalGrid->setConfiguration(toroidalConfig);
    physicsSystem.installBroadphase(toroidalGrid);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase3_init();
    CollectTimeAndMemoryData("toroidal3");
    physicsSystem.shutdown();


    //hashgrid
    hashGridConfig.baseSize = 1.5;
    hashGridConfig.nBucket = 1024;
    hashGridConfig.nLevel = 3;
    hashGridConfig.sizeMul = 8;
    ftHierarchicalGrid* hierarchicalGrid = new ftHierarchicalGrid();
    broadphaseSystem = hierarchicalGrid;
    hierarchicalGrid->setConfiguration(hashGridConfig);
    physicsSystem.installBroadphase(hierarchicalGrid);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase3_init();
    CollectTimeAndMemoryData("hierarchical3");
    physicsSystem.shutdown();

    //quad
    quadConfig.maxLevel = 15;
    quadConfig.worldAABB.min = ftVector2(-50,-50);
    quadConfig.worldAABB.max = ftVector2(4050,4050);
    ftQuadTree* quadTree = new ftQuadTree();
    broadphaseSystem = quadTree;
    quadTree->setConfiguration(quadConfig);
    physicsSystem.installBroadphase(quadTree);
    physicsSystem.setConfiguration(physicsConfig);
    physicsSystem.init();
    TestCase3_init();
    CollectTimeAndMemoryData("Quad3");
    physicsSystem.shutdown();
}

void CollectTimeAndMemoryData(std::string filename) {
    float time1[N_TEST_STEP], time2[N_TEST_STEP];
    uint64 memory[N_TEST_STEP];

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

    for (int iter = 1; iter < N_TEST_SAMPLE; ++iter) {
        cout<<"Iteration : "<<iter + 1<<std::endl;
        for (int i = 0; i < N_TEST_STEP; ++i) {
            ftBenchmark::BeginFrame();
            physicsSystem.step(1.0/60);
            ftBenchmark::EndFrame();
            time1[i] += ftBenchmark::benchTables[0].data[i];
            time2[i] += ftBenchmark::benchTables[1].data[i];
        }
        ftBenchmark::Clear();
    }

    for (int i = 0; i < N_TEST_STEP; ++i) {
        time1[i] /= N_TEST_SAMPLE;
        time2[i] /= N_TEST_SAMPLE;
    }

    //write to file
    ofstream file(filename);
    if (file.is_open()) {
        for (int i = 0; i < N_TEST_STEP; ++i) {
            file << time1[i]<<" "<<time2[i]<<" "<<memory[i]<<std::endl;
        }
        file.close();
    } else {
        cout<<"fail to open file";
    }
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
    createStaticBox(ftVector2(-50,25), 0,ftVector2(0.5,25), 0.2);
    createStaticBox(ftVector2(50,25), 0,ftVector2(0.5,25), 0.2);

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
    int32 spacing = 500;
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
    for (int i = 0; i < 50 ; ++i) {
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
    for (int i = 0; i < 30; ++i) {
        real xOffset = i * 160;
        createPendulum(ftVector2(xOffset + 25, 15));
        createStaticBox(ftVector2(xOffset + 25, 4), 0, ftVector2(25, 4), 0.6);
        createStaticBox(ftVector2(xOffset + 60, 10), 0, ftVector2(10, 10), 0.6);
        createPendulum(ftVector2(xOffset + 80, 47));
        createStaticBox(ftVector2(xOffset + 80, 20), 0, ftVector2(10, 20), 0.6);
        createStaticBox(ftVector2(xOffset + 100, 30), 0, ftVector2(10, 30), 0.6);
        createStaticBox(ftVector2(xOffset + 140, 56), 0, ftVector2(10, 4), 0.6);
    }
}
