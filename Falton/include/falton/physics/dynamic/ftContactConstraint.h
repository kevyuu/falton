//
// Created by Kevin Yu on 4/16/16.
//

#ifndef FALTON_FTCONTACTCONSTRAINT_H
#define FALTON_FTCONTACTCONSTRAINT_H

#include <falton/physics/joint/ftJoint.h>
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

    friend class ftConstraintSolver;

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

};

struct ftConstraintGroup {

    ftBody** bodies;
    ftVector2* positions = nullptr;
    ftVector2* velocities = nullptr;

    real* angularVelocities;

    ftContactConstraint* constraints;
    ftJoint** joints;

    uint32 nConstraint;
    uint32 nBody;
    uint32 nJoint;

};

#endif //FALTON_FTCONTACTCONSTRAINT_H
