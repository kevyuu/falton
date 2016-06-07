//
// Created by Kevin Yu on 4/14/16.
//

#ifndef FALTON_FTPHYSICSSYSTEM_H
#define FALTON_FTPHYSICSSYSTEM_H

#include <falton/math/precision.h>
#include <falton/physics/ftConstraintSolver.h>
#include <falton/physics/shape/ftShapeBuffer.h>
#include <falton/physics/dynamic/ftIslandSystem.h>

struct ftBody;
struct ftCollider;
class ftShape;
class ftPivotJoint;
class ftDistanceJoint;


typedef void (*ftBodyIterFunc) (ftBody* body, void* data);

class  ftPhysicsSystem {

public:

    struct ftConfig {

        real sleepTimeLimit = 0.05f;
        real sleepLinearLimit = 0.08f;
        real sleepAngularLimit = (2.0f / 180.0f * PI);;
        ftVector2 gravity = {0,-10};

        ftConstraintSolver::ftConfig solverConfig;
    };

    void setConfiguration(const ftConfig& config);

    void init();
    void shutdown();

    void installBroadphase(ftBroadphaseSystem* broadphase);

    ftBody* createStaticBody(const ftVector2& position, real orientation);
    ftBody* createKinematicBody(const ftVector2& position, real orientation);
    ftBody* createDynamicBody(const ftVector2& position, real orientation, real mass, real moment);
    void destroyBody(ftBody* body);
    void updateBody(ftBody* body); //call this every time user change body member variable.

    ftCollider* createCollider(ftBody* body, ftShape* shape, const ftVector2& position, real orientation);
    void destroyCollider(ftCollider* collider);

    ftPivotJoint* createPivotJoint(ftBody *bodyA, ftBody *bodyB, ftVector2 anchorPoint);
    ftDistanceJoint* createDistanceJoint(ftBody* bodyA, ftBody* bodyB, ftVector2 localAnchorA, ftVector2 localAnchorB);

    void iterateBody(ftBodyIterFunc iterFunc, void* data);
    void iterateStaticBody(ftBodyIterFunc iterFunc, void* data);
    void iterateKinematicBody(ftBodyIterFunc iterFunc, void* data);
    void iterateDynamicBody(ftBodyIterFunc iterFunc, void* data);

    template <typename T>
    void forEachBody(T func);
    template <typename T>
    void forEachStaticBody(T func);
    template <typename T>
    void forEachKinematicBody(T func);
    template <typename T>
    void forEachDynamicBody(T func);

    void step(real dt);

private:
    ftBodyBuffer m_staticBodies;
    ftBodyBuffer m_kinematicBodies;
    ftBodyBuffer m_dynamicBodies;
    ftBodyBuffer m_sleepingBodies;

    ftShapeBuffer m_shapeBuffer;

    ftConstraintSolver m_contactSolver;
    ftCollisionSystem m_collisionSystem;
    ftIslandSystem m_islandSystem;

    ftBroadphaseSystem *m_broadphase;

    ftChunkArray<ftJoint*> m_joints;

    //configuration
    real m_sleepTimeLimit = 0.5f;
    real m_sleepLinearLimit = 0.08f;
    real m_sleepAngualrLimit = (2.0f / 180.0f * PI);
    ftVector2 m_gravity = {0, -10};

    ftShape* createShape(ftShape* shape);
    void destroyShape(ftShape* shape);

    void integrateVelocity(real dt);
    void integratePosition(real dt);

    void updateBodiesActivation(real dt);

    static void beginContactListener(ftContact* contact, void* data);
    static void updateContactListener(ftContact* contact, void* data);
    static void endContactListener(ftContact* contact, void* data);

};

template <typename T>
inline void ftPhysicsSystem::forEachBody(T func) {
    m_staticBodies.forEach(func);
    m_kinematicBodies.forEach(func);
    m_dynamicBodies.forEach(func);
    m_sleepingBodies.forEach(func);
}

template <typename T>
inline void ftPhysicsSystem::forEachStaticBody(T func) {
    m_staticBodies.forEach(func);
}

template <typename T>
inline void ftPhysicsSystem::forEachKinematicBody(T func) {
    m_kinematicBodies.forEach(func);
}

template <typename T>
inline void ftPhysicsSystem::forEachDynamicBody(T func) {
    m_dynamicBodies.forEach(func);
    m_sleepingBodies.forEach(func);
}


#endif //FALTON_FTPHYSICSSYSTEM_H
