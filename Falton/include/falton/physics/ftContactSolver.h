//
// Created by Kevin Yu on 3/22/16.
//

#ifndef FALTON_FTCONTACTCONSTRAINTSOLVER_H
#define FALTON_FTCONTACTCONSTRAINTSOLVER_H

#include <falton/physics/Collision/ftCollisionSystem.h>
#include <falton/physics/dynamic/ftContactConstraint.h>
#include "falton/physics/dynamic/ftBody.h"
#include "falton/physics/dynamic/ftCollider.h"
#include "falton/math/type.h"

struct ftContactConstraint;
 struct ftContactConstraintGroup;
struct ftContact;
struct ftIsland;

struct ftContactSolverOption {
    uint8 numIteration;
    real baumgarteCoef;
    real allowedPenetration;
};

class ftContactSolver {
public:

    void init(const ftContactSolverOption& option);
    void shutdown();

    void warmStart();
    void solve(real dt);

    void createConstraints(const ftIsland& island);
    void clearConstraints();

private:
    void createContactConstraint(ftCollider *colliderA, ftCollider *colliderB,
                                 ftContact *contact, ftContactConstraint *constraint);

    ftContactConstraintGroup m_constraintGroup;
    ftContactSolverOption m_option;

};


#endif //FALTON_FTCONTACTCONSTRAINTSOLVER_H
