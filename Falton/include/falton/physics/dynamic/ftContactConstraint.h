//
// Created by Kevin Yu on 4/16/16.
//

#ifndef FALTON_FTCONTACTCONSTRAINT_H
#define FALTON_FTCONTACTCONSTRAINT_H

#include "falton/math/math.h"

struct ftBody;
struct ftContact;
struct ftCollider;

struct ftContactPointConstraint {

    real normalMass;
    real tangentMass;

    real positionBias;
    real restitutionBias;

    real nIAcc = 0; //impulse normal accumulation for penetration solver
    real tIAcc = 0; //impulse tangent accumulation for friction solver

    ftVector2 r1;
    ftVector2 r2;

    friend class ftContactSolver;

};

struct ftContactConstraint {

    uint32 bodyIDA, bodyIDB;
    ftVector2 normal;
    ftContactPointConstraint pointConstraint[2];
    ftContact *contact = nullptr;

    real frictionCoef;
    uint8 numContactPoint;

    real invMassA;
    real invMassB;
    real invMomentA;
    real invMomentB;

    friend class ftContactSolver;

};

struct ftContactConstraintGroup {

    ftBody** bodies;
    ftVector2* positions = nullptr;
    ftVector2* velocities = nullptr;

    real* angularVelocities;

    ftContactConstraint* constraints;
    uint32 numConstraint;
    uint32 numBody;
    friend class ftContactSolver;

};

#endif //FALTON_FTCONTACTCONSTRAINT_H
