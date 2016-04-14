//
// Created by Kevin Yu on 3/22/16.
//

#ifndef FALTON_FTCONTACTCONSTRAINTSOLVER_H
#define FALTON_FTCONTACTCONSTRAINTSOLVER_H

#include <falton/physics/Collision/ftCollisionSystem.h>
#include "falton/physics/ftBody.h"
#include "falton/physics/ftCollider.h"
#include "falton/math/type.h"

struct ftContactSolverOption {
    uint8 numIteration;
    real baumgarteCoef;
    real allowedPenetration;
};

struct ftContactPointConstraint {

    real normalMass;
    real tangentMass;
    real positionBias;

    real accumNormalImpulse;
    real accumTangentImpulse;

    ftVector2 r1;
    ftVector2 r2;

};

struct ftContactConstraint {

    ftBody *bodyA, *bodyB;
    ftVector2 normal;
    ftContactPointConstraint pointConstraint[2];
    float frictionCoef;
    uint8 numContactPoint;

};


class ftContactSolver {

public:
    ftContactSolverOption option;

    ftContactSolver(const ftContactSolverOption& option);

    ~ftContactSolver();

    ftChunkArray<ftContactConstraint> *constraintBuffer;
    ftChunkArray<ftContactConstraint> *constraintCache;

    void addConstraint(ftCollider* colliderA, ftCollider* colliderB, ftContact* contact);

    void updateConstraint(ftCollider* colliderA, ftCollider* collider, ftContact* contact);

    void warmStart();
    void solve(real dt);

private:
    void createContactConstraint(ftCollider *colliderA, ftCollider *colliderB,
                                 ftManifold *manifold, ftContactConstraint *constraint);

};


#endif //FALTON_FTCONTACTCONSTRAINTSOLVER_H
