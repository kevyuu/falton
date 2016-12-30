#pragma once

#include "falton/setting.h"
#include "falton/math.h"
#include "falton/dynamic/ftJoint.h"
class ftJoint;

/* Role : System */
class ftJointSolver
{

public:
  static void preSolve(ftJoint *joint, real dt);
  static void warmStart(ftJoint *joint,
                        ftVector2 *vArray,
                        real *wArray);
  static void solve(ftJoint *joint,
                    ftVector2 *vArray,
                    real *wArray);

private:
  typedef void (*ftPreSolveFunc)(ftJoint *joint, real dt);
  typedef void (*ftWarmStartFunc)(ftJoint *joint,
                                  ftVector2 *vArray,
                                  real *wArray);
  typedef void (*ftSolveFunc)(ftJoint *joint,
                              ftVector2 *vArray,
                              real *wArray);

  struct ftJointFunc
  {
    ftPreSolveFunc preSolve;
    ftWarmStartFunc warmStart;
    ftSolveFunc solve;
  };

  static ftJointFunc jointFunc[ftJoint::COUNT_JOINT_TYPE];

  // Distance Joint
  static void preSolveDistanceJoint(ftJoint *joint, real dt);
  static void warmStartDistanceJoint(ftJoint *joint,
                                     ftVector2 *vArray,
                                     real *wArray);
  static void solveDistanceJoint(ftJoint *joint,
                                 ftVector2 *vArray,
                                 real *wArray);

  // Dynamo Joint
  static void preSolveDynamoJoint(ftJoint *joint, real dt);
  static void warmStartDynamoJoint(ftJoint *joint,
                                   ftVector2 *vArray,
                                   real *wArray);
  static void solveDynamoJoint(ftJoint *joint,
                               ftVector2 *vArray,
                               real *wArray);

  // Hinge Joint
  static void preSolveHingeJoint(ftJoint *joint, real dt);
  static void warmStartHingeJoint(ftJoint *joint,
                                  ftVector2 *vArray,
                                  real *wArray);
  static void solveHingeJoint(ftJoint *joint,
                              ftVector2 *vArray,
                              real *wArray);

  // Piston Joint
  static void preSolvePistonJoint(ftJoint *joint, real dt);
  static void warmStartPistonJoint(ftJoint *joint,
                                   ftVector2 *vArray,
                                   real *wArray);
  static void solvePistonJoint(ftJoint *joint,
                               ftVector2 *vArray,
                               real *wArray);

  // Spring Joint
  static void preSolveSpringJoint(ftJoint *joint, real dt);
  static void warmStartSpringJoint(ftJoint *joint,
                                   ftVector2 *vArray,
                                   real *wArray);
  static void solveSpringJoint(ftJoint *joint,
                               ftVector2 *vArray,
                               real *wArray);
};