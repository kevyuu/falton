//
// Created by Kevin Yu on 3/22/16.
//

#ifndef FALTON_FTCONTACTCONSTRAINTSOLVER_H
#define FALTON_FTCONTACTCONSTRAINTSOLVER_H

#include <falton/collision/ftCollisionSystem.h>
#include <falton/dynamic/ftContactConstraint.h>
#include <falton/dynamic/ftBody.h>
#include <falton/dynamic/ftCollider.h>

struct ftContactConstraint;
struct ftConstraintGroup;
struct ftContact;
struct ftIsland;


class ftConstraintSolver {
public:

    struct ftConfig {
        uint8 numIteration = 10;
        real baumgarteCoef = 0.2;
        real allowedPenetration = 0.01;
    };

    void setConfiguration(const ftConfig& config);
    void init();
    void shutdown();

    void warmStart();
    void preSolve(real dt);
    void solve(real dt);

    void createConstraints(const ftIsland& island);
    void clearConstraints();

private:
    void createContactConstraint(ftCollider *colliderA, 
                                 ftCollider *colliderB,
                                 ftContact *contact, 
                                 ftContactConstraint *constraint);

    ftConstraintGroup m_constraintGroup;
    ftConfig m_option;

};

#endif //FALTON_FTCONTACTCONSTRAINTSOLVER_H
