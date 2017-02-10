//
// Created by Kevin Yu on 4/16/16.
//
#pragma once

#include <falton/dynamic/ftJoint.h>
#include <falton/math.h>

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

    int32 bodyIDA, bodyIDB;
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
    ftVector2* velocities = nullptr;

    real* angularVelocities;

    ftContactConstraint* constraints;
    ftJoint** joints;

    int32 nConstraint;
    int32 nBody;
    int32 nJoint;

};