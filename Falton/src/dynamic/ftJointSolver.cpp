#include "falton/dynamic/ftJointSolver.h"
#include "falton/dynamic/ftJoint.h"
#include "falton/dynamic/ftBody.h"

static float s_baumgarteCoef = 0.2;
static float s_linearSlop = 0.01;
static float s_angularSlop = 0.01;

ftJointSolver::ftJointFunc ftJointSolver::jointFunc[] = {
    {
        ftJointSolver::preSolveDistanceJoint,
        ftJointSolver::warmStartDistanceJoint,
        ftJointSolver::solveDistanceJoint
    },
    {
        ftJointSolver::preSolveDynamoJoint,
        ftJointSolver::warmStartDynamoJoint,
        ftJointSolver::solveDynamoJoint
    },
    {
        ftJointSolver::preSolveHingeJoint,
        ftJointSolver::warmStartHingeJoint,
        ftJointSolver::solveHingeJoint
    },
    {
        ftJointSolver::preSolvePistonJoint,
        ftJointSolver::warmStartPistonJoint,
        ftJointSolver::solvePistonJoint
    },
    {
        ftJointSolver::preSolveSpringJoint,
        ftJointSolver::warmStartSpringJoint,
        ftJointSolver::solveSpringJoint
    }
};

void ftJointSolver::preSolve(ftJoint* joint, real dt) {
    jointFunc[joint->jointType].preSolve(joint, dt);
}

void ftJointSolver::warmStart(ftJoint* joint, ftVector2* vArray, real* wArray) {
    jointFunc[joint->jointType].warmStart(joint, vArray, wArray);
}

void ftJointSolver::solve(ftJoint* joint, ftVector2* vArray, real* wArray) {
    jointFunc[joint->jointType].solve(joint, vArray, wArray);
}

// Distance Joint
void ftJointSolver::preSolveDistanceJoint(ftJoint *joint, real dt)
{ 
    ftDistanceJoint *dist = (ftDistanceJoint *)joint;

    ftBody *bodyA = dist->bodyA;
    ftBody *bodyB = dist->bodyB;
    ftVector2 localAnchorA = dist->localAnchorA;
    ftVector2 localAnchorB = dist->localAnchorB;

    dist->jVa = (bodyA->transform * localAnchorA -
                 bodyB->transform * localAnchorB);

    dist->bodyIDA = bodyA->islandId;
    dist->invMassA = bodyA->inverseMass;
    dist->invMomentA = bodyA->inverseMoment;
    ftVector2 rA = bodyA->transform.rotation *
                   (localAnchorA - bodyA->centerOfMass);

    dist->bodyIDB = bodyB->islandId;
    dist->invMassB = bodyB->inverseMass;
    dist->invMomentB = bodyB->inverseMoment;
    ftVector2 rB = bodyB->transform.rotation *
                   (localAnchorB - bodyB->centerOfMass);

    dist->jWa = rA.cross(dist->jVa);
    dist->jWb = rB.cross(dist->jVa);

    real squareMagnitude = dist->jVa.square_magnitude();
    real k = squareMagnitude * dist->invMassA +
             squareMagnitude * dist->invMassB +
             dist->invMomentA * dist->jWa * dist->jWa +
             dist->invMomentB * dist->jWb * dist->jWb;
    dist->invK = 1 / k;

    real curDistance = dist->jVa.magnitude();
    real positionDrift = curDistance - dist->distance;
    if (ftAbs(positionDrift) < s_linearSlop) positionDrift = 0;
    dist->positionBias = s_baumgarteCoef * positionDrift / dt;
}

void ftJointSolver::warmStartDistanceJoint(ftJoint *joint,
                                           ftVector2 *vArray,
                                           real *wArray)
{
    ftDistanceJoint *dist = (ftDistanceJoint *)joint;

    ftVector2 linearI = dist->jVa * dist->iAcc / 60;
    real angularIA = dist->jWa * dist->iAcc / 60;
    real angularIB = dist->jWb * dist->iAcc / 60;

    vArray[dist->bodyIDA] += dist->invMassA * linearI;
    vArray[dist->bodyIDB] -= dist->invMassB * linearI;

    wArray[dist->bodyIDA] += dist->invMomentA * angularIA;
    wArray[dist->bodyIDB] -= dist->invMomentB * angularIB;
}

void ftJointSolver::solveDistanceJoint(ftJoint *joint,
                                       ftVector2 *vArray,
                                       real *wArray)
{
    ftDistanceJoint *dist = (ftDistanceJoint *)joint;
    int32 bodyIDA = dist->bodyIDA;
    int32 bodyIDB = dist->bodyIDB;
    ftVector2 jVa = dist->jVa;
    real jWa = dist->jWa;
    real jWb = dist->jWb;
    real invMassA = dist->invMassA;
    real invMassB = dist->invMassB;
    real invMomentA = dist->invMomentA;
    real invMomentB = dist->invMomentB;

    ftVector2 vA = vArray[bodyIDA];
    ftVector2 vB = vArray[bodyIDB];
    real wA = wArray[bodyIDA];
    real wB = wArray[bodyIDB];

    real jv = jVa.dot(vA) - jVa.dot(vB) + jWa * wA - jWb * wB;

    real impulse = dist->invK * (-jv - dist->positionBias);
    dist->iAcc += impulse;

    ftVector2 linearI = jVa * impulse;
    real angularIA = jWa * impulse;
    real angularIB = jWb * impulse;

    vArray[bodyIDA] += invMassA * linearI;
    vArray[bodyIDB] -= invMassB * linearI;

    wArray[bodyIDA] += invMomentA * angularIA;
    wArray[bodyIDB] -= invMomentB * angularIB;
}
// End of Distance Joint

// Dynamo Joint
void ftJointSolver::preSolveDynamoJoint(ftJoint *joint, real dt)
{

    ftDynamoJoint *dynamo = (ftDynamoJoint *)joint;
    ftBody *bodyA = dynamo->bodyA;
    ftBody *bodyB = dynamo->bodyB;

    dynamo->invMomentA = bodyA->inverseMoment;
    dynamo->invMomentB = bodyB->inverseMoment;
    dynamo->invK = 1 / (dynamo->invMomentA + dynamo->invMomentB);

    dynamo->bodyIDA = bodyA->islandId;
    dynamo->bodyIDB = bodyB->islandId;
    dynamo->maxImpulse = dynamo->maxTorque * dt;
}

void ftJointSolver::warmStartDynamoJoint(ftJoint *joint,
                                         ftVector2 *vArray,
                                         real *wArray)
{

    ftDynamoJoint *dynamo = (ftDynamoJoint *)joint;
    int32 bodyIDA = dynamo->bodyIDA;
    int32 bodyIDB = dynamo->bodyIDB;
    real invMomentA = dynamo->invMomentA;
    real invMomentB = dynamo->invMomentB;
    real iAcc = dynamo->iAcc;

    wArray[bodyIDA] -= invMomentA * iAcc;
    wArray[bodyIDB] += invMomentB * iAcc;
}

void ftJointSolver::solveDynamoJoint(ftJoint *joint,
                                     ftVector2 *vArray,
                                     real *wArray)
{

    ftDynamoJoint *dynamo = (ftDynamoJoint *)joint;
    int32 bodyIDA = dynamo->bodyIDA;
    int32 bodyIDB = dynamo->bodyIDB;
    real invMomentA = dynamo->invMomentA;
    real invMomentB = dynamo->invMomentB;

    real jV = wArray[bodyIDB] - wArray[bodyIDA] - dynamo->targetRate;

    real impulse = dynamo->invK * -jV;

    real oldI = dynamo->iAcc;
    dynamo->iAcc += impulse;
    if (dynamo->iAcc > dynamo->maxImpulse)
    {
        dynamo->iAcc = dynamo->maxImpulse;
    }
    impulse = dynamo->iAcc - oldI;

    wArray[bodyIDB] += invMomentB * impulse;
    wArray[bodyIDA] -= invMomentA * impulse;
}
// End of Dynamo Joint

// Hinge Joint
void ftJointSolver::preSolveHingeJoint(ftJoint *joint,
                                       real dt)
{
    ftHingeJoint *hinge = (ftHingeJoint *)joint;
    ftBody *bodyA = hinge->bodyA;
    ftBody *bodyB = hinge->bodyB;

    hinge->bodyIDA = bodyA->islandId;
    hinge->bodyIDB = bodyB->islandId;

    hinge->invMassA = bodyA->inverseMass;
    hinge->invMomentA = bodyA->inverseMoment;
    hinge->invMassB = bodyB->inverseMass;
    hinge->invMomentB = bodyB->inverseMoment;

    hinge->rA = bodyA->transform.rotation *
                (hinge->localAnchorA - bodyA->centerOfMass);
    hinge->rB = bodyB->transform.rotation *
                (hinge->localAnchorB - bodyB->centerOfMass);

    ftVector2 rA = hinge->rA;
    ftVector2 rB = hinge->rB;
    real invMassA = hinge->invMassA;
    real invMassB = hinge->invMassB;
    real invMomentA = hinge->invMomentA;
    real invMomentB = hinge->invMomentB;
    hinge->invKDistance.element[0][0] = invMassA + invMassB +
                                        invMomentA * rA.y * rA.y +
                                        invMomentB * rB.y * rB.y;
    hinge->invKDistance.element[0][1] = -invMomentA * rA.y * rA.x -
                                        invMomentB * rB.y * rB.x;
    hinge->invKDistance.element[1][0] = hinge->invKDistance.element[0][1];
    hinge->invKDistance.element[1][1] = invMassA + invMassB +
                                        invMomentA * rA.x * rA.x +
                                        invMomentB * rB.x * rB.x;
    hinge->invKDistance.invert();

    ftVector2 worldAnchorA = bodyA->transform * hinge->localAnchorA;
    ftVector2 worldAnchorB = bodyB->transform * hinge->localAnchorB;
    ftVector2 positionDrift = worldAnchorA - worldAnchorB;
    if (positionDrift.x * positionDrift.x + positionDrift.y * positionDrift.y < s_linearSlop * s_linearSlop) {
        positionDrift = ftVector2(0, 0);
    }
    hinge->dBias = positionDrift * 0.2f / dt;

    hinge->fMaxImpulse = hinge->torqueFriction * dt;
    hinge->invKFriction = 1 / (invMomentA + invMomentB);

    if (hinge->enableLimit) {
        real diffAngle = bodyB->transform.rotation.angle - bodyA->transform.rotation.angle;
        real minAngle = hinge->lowerLimit;
        real maxAngle = hinge->upperLimit;
        hinge->invKLimit = 1 / (invMomentA + invMomentB);
        real angleDrift = 0;
        if (diffAngle < minAngle) {
            angleDrift = minAngle - diffAngle;
        } else if (diffAngle > maxAngle) {
            angleDrift = maxAngle - diffAngle;
        } else {
            hinge->limitImpulseAcc = 0;
        }
        hinge->limitBias = -1 * s_baumgarteCoef * angleDrift / dt;
    }

}

void ftJointSolver::warmStartHingeJoint(ftJoint *joint,
                                        ftVector2 *vArray,
                                        real *wArray)
{
    ftHingeJoint *hinge = (ftHingeJoint *)joint;
    int32 bodyIDA = hinge->bodyIDA;
    int32 bodyIDB = hinge->bodyIDB;
    real invMassA = hinge->invMassA;
    real invMassB = hinge->invMassB;
    real invMomentA = hinge->invMomentA;
    real invMomentB = hinge->invMomentB;
    ftVector2 rA = hinge->rA;
    ftVector2 rB = hinge->rB;

    //distance constraint
    {
        ftVector2 impulse = hinge->dIAcc;
        vArray[bodyIDA] -= (invMassA * impulse);
        wArray[bodyIDA] -= (invMomentA * rA.cross(impulse));

        vArray[bodyIDB] += (invMassB * impulse);
        wArray[bodyIDB] += (invMomentB * rB.cross(impulse));
    }

    // friction constraint
    {
        real impulse = hinge->fIAcc;

        wArray[bodyIDB] += invMomentB * impulse;
        wArray[bodyIDA] -= invMomentA * impulse;
    }

    // limit constraint
    {
        if (hinge->enableLimit) {
            wArray[bodyIDB] += invMomentB * hinge->limitImpulseAcc;
            wArray[bodyIDA] -= invMomentA * hinge->limitImpulseAcc;
        }
    }
}

void ftJointSolver::solveHingeJoint(ftJoint *joint,
                                    ftVector2 *vArray,
                                    real *wArray)
{

    ftHingeJoint *hinge = (ftHingeJoint *)joint;
    int32 bodyIDA = hinge->bodyIDA;
    int32 bodyIDB = hinge->bodyIDB;
    real invMassA = hinge->invMassA;
    real invMassB = hinge->invMassB;
    real invMomentA = hinge->invMomentA;
    real invMomentB = hinge->invMomentB;
    ftVector2 rA = hinge->rA;
    ftVector2 rB = hinge->rB;

    //distance constraint
    {
        ftVector2 jv = vArray[bodyIDB] +
                       rB.invCross(wArray[bodyIDB]) -
                       vArray[bodyIDA] -
                       rA.invCross(wArray[bodyIDA]);

        ftVector2 impulse = hinge->invKDistance * (-1 * (jv - hinge->dBias));
        
        hinge->dIAcc += impulse;

        vArray[bodyIDA] -= (invMassA * impulse);
        wArray[bodyIDA] -= (invMomentA * rA.cross(impulse));

        vArray[bodyIDB] += (invMassB * impulse);
        wArray[bodyIDB] += (invMomentB * rB.cross(impulse));

    }

    //friction constraint
    {
        real jv = wArray[bodyIDB] - wArray[bodyIDA];
        real impulse = hinge->invKFriction * (jv * -1);
        real oldI = hinge->fIAcc;
        hinge->fIAcc += impulse;
        if (hinge->fIAcc > hinge->fMaxImpulse)
        {
            hinge->fIAcc = hinge->fMaxImpulse;
        }
        else if (hinge->fIAcc < -hinge->fMaxImpulse)
        {
            hinge->fIAcc = -hinge->fMaxImpulse;
        }
        impulse = hinge->fIAcc - oldI;

        wArray[bodyIDB] += invMomentB * impulse;
        wArray[bodyIDA] -= invMomentA * impulse;
    }

    //angle limit constraint
    if (hinge->enableLimit) {
        if (hinge->limitBias == 0) return;

        real jv = wArray[bodyIDB] - wArray[bodyIDA];
        real impulse = hinge->invKLimit * ((jv + hinge->limitBias) * -1);
        real oldImpulse = hinge->limitImpulseAcc;
        if (hinge->limitBias < 0) {
            hinge->limitImpulseAcc = ftMax(oldImpulse + impulse, 0);
        } else {
            hinge->limitImpulseAcc = ftMin(oldImpulse + impulse, 0);
        }
        impulse = hinge->limitImpulseAcc - oldImpulse;
        wArray[bodyIDB] += invMomentB * impulse;
        wArray[bodyIDA] -= invMomentA * impulse;

    }
}
// End of Hinge Joint

// Piston Joint
void ftJointSolver::preSolvePistonJoint(ftJoint *joint, real dt)
{
    ftPistonJoint *piston = (ftPistonJoint *)joint;
    ftBody *bodyA = piston->bodyA;
    ftBody *bodyB = piston->bodyB;

    piston->bodyIDA = bodyA->islandId;
    piston->bodyIDB = bodyB->islandId;

    piston->invMassA = bodyA->inverseMass;
    piston->invMomentA = bodyA->inverseMoment;
    piston->invMassB = bodyB->inverseMass;
    piston->invMomentB = bodyB->inverseMoment;
    piston->tAxis = (bodyA->transform.rotation * piston->localAxis).perpendicular();
    
    piston->invKRot = 1 / (piston->invMomentA + piston->invMomentB);
    real diffAngle = bodyB->transform.rotation.angle - bodyA->transform.rotation.angle;
    if (fabs(diffAngle) < s_angularSlop) {
        diffAngle = 0;
    }
    piston->rotBias = 1 * s_baumgarteCoef * (diffAngle - piston->refAngle);
    

    piston->rA = bodyA->transform.rotation *
                 (piston->localAnchorA - bodyA->centerOfMass);
    piston->rB = bodyB->transform.rotation *
                 (piston->localAnchorB - bodyB->centerOfMass);
    real kTrans = piston->invMassA + piston->invMassB;
    real rtA = piston->rA.cross(piston->tAxis);
    real rtB = piston->rB.cross(piston->tAxis);
    kTrans += (piston->invMomentA * rtA * rtA + piston->invMomentB * rtB * rtB);
    piston->invKTrans = 1 / kTrans;

    ftVector2 worldAnchorA = bodyA->transform * piston->localAnchorA;
    ftVector2 worldAnchorB = bodyB->transform * piston->localAnchorB;
    real separation = (worldAnchorB - worldAnchorA).dot(piston->tAxis);
    if (fabs(separation) < s_linearSlop) {
        separation = 0;
    }
    piston->transBias = -1 * s_baumgarteCoef * (separation / dt);

}

void ftJointSolver::warmStartPistonJoint(ftJoint *joint,
                                         ftVector2 *vArray,
                                         real *wArray)
{
    ftPistonJoint* piston = (ftPistonJoint*) joint;
    int bodyIDA = piston->bodyIDA;
    int bodyIDB = piston->bodyIDB;

    // translation constraint
    {
        real rtA = piston->rA.cross(piston->tAxis);
        real rtB = piston->rB.cross(piston->tAxis);
        real angularIA = rtA * piston->transImpulseAcc;
        real angularIB = rtB * piston->transImpulseAcc;
        ftVector2 linearI = piston->tAxis * piston->transImpulseAcc;

        vArray[piston->bodyIDA] -= (piston->invMassA * linearI);
        wArray[piston->bodyIDA] -= (piston->invMomentA * angularIA);

        vArray[piston->bodyIDB] += (piston->invMassB * linearI);
        wArray[piston->bodyIDB] += (piston->invMomentB * angularIB);
    }

    // rotation constraint
    {
        wArray[piston->bodyIDB] += piston->invMomentB * piston->rotImpulseAcc;
        wArray[piston->bodyIDA] -= piston->invMomentA * piston->rotImpulseAcc;
    }
}

void ftJointSolver::solvePistonJoint(ftJoint *joint,
                                     ftVector2 *vArray,
                                     real *wArray)
{
    ftPistonJoint *piston = (ftPistonJoint *)joint;
    int32 bodyIDA = piston->bodyIDA;
    int32 bodyIDB = piston->bodyIDB;

    // solve translation constraint
    {
        ftVector2 relV = vArray[bodyIDB] +
                       piston->rB.invCross(wArray[piston->bodyIDB]) -
                       vArray[bodyIDA] -
                       piston->rA.invCross(wArray[piston->bodyIDB]);
        real jv = relV.dot(piston->tAxis);

        real lambda = -(jv - piston->transBias)* piston->invKTrans;

        piston->transImpulseAcc += lambda;

        real rtA = piston->rA.cross(piston->tAxis);
        real rtB = piston->rB.cross(piston->tAxis);
        real angularIA = rtA * lambda;
        real angularIB = rtB * lambda;
        ftVector2 linearI = piston->tAxis * lambda;

        vArray[piston->bodyIDA] -= (piston->invMassA * linearI);
        wArray[piston->bodyIDA] -= (piston->invMomentA * angularIA);

        vArray[piston->bodyIDB] += (piston->invMassB * linearI);
        wArray[piston->bodyIDB] += (piston->invMomentB * angularIB);
    }

    // solve rotation constraint
    {
        real jv = wArray[piston->bodyIDB] - wArray[piston->bodyIDA];
        real impulse = piston->invKRot * -(jv + piston->rotBias);
        piston->rotImpulseAcc += impulse;

        wArray[piston->bodyIDB] += piston->invMomentB * impulse;
        wArray[piston->bodyIDA] -= piston->invMomentA * impulse;
    }
}
// End of Piston Joint

// Spring Joint
void ftJointSolver::preSolveSpringJoint(ftJoint *joint, real dt)
{
    ftSpringJoint *spring = (ftSpringJoint *)joint;
    ftBody *bodyA = spring->bodyA;
    ftBody *bodyB = spring->bodyB;

    spring->bodyIDA = bodyA->islandId;
    spring->bodyIDB = bodyB->islandId;

    spring->invMassA = bodyA->inverseMass;
    spring->invMomentA = bodyA->inverseMoment;
    spring->invMassB = bodyB->inverseMass;
    spring->invMomentB = bodyB->inverseMoment;

    spring->invKRot = 1 / (spring->invMomentA + spring->invMomentB);

    spring->rA = bodyA->transform.rotation *
                 (spring->localAnchorA - bodyA->centerOfMass);
    spring->rB = bodyB->transform.rotation *
                 (spring->localAnchorB - bodyB->centerOfMass);
    real kTrans = spring->invMassA + spring->invMassB;
    real rtA = spring->rA.cross(spring->tAxis);
    real rtB = spring->rB.cross(spring->tAxis);
    kTrans += (spring->invMomentA * rtA * rtA + spring->invMomentB * rtB * rtB);
    spring->invKTrans = 1 / kTrans;

    ftVector2 anchorDiff = bodyB->transform * spring->localAnchorA - bodyA->transform * spring->localAnchorB;
    spring->springAxis = anchorDiff.unit();
    spring->tAxis = spring->springAxis.perpendicular();

    //apply spring force
    {
        ftVector2 worldAnchorA = bodyA->transform * spring->localAnchorA;
        ftVector2 worldAnchorB = bodyB->transform * spring->localAnchorB;
        real dist = (worldAnchorB - worldAnchorA).dot(spring->springAxis);
        real forceMagnitude = (spring->restLength - dist) * spring->stiffness;
        real impulseMagnitude = forceMagnitude * dt;
        ftVector2 impulse = spring->springAxis * impulseMagnitude;
        bodyB->velocity += (spring->invMassB * impulse);
        bodyA->velocity -= (spring->invMassA * impulse);
        bodyB->angularVelocity += (spring->invMomentB * impulse.cross(spring->rB));
        bodyA->angularVelocity -= (spring->invMomentA * impulse.cross(spring->rA));
    }
}

void ftJointSolver::warmStartSpringJoint(ftJoint *joint,
                                         ftVector2 *vArray,
                                         real *wArray)
{

    ftSpringJoint *spring = (ftSpringJoint *)joint;
    int32 bodyIDA = spring->bodyIDA;
    int32 bodyIDB = spring->bodyIDB;

    // translation constraint
    {
        ftVector2 linearI = spring->tAxis * spring->translationIAcc;
        real angularIA = spring->rA.cross(spring->tAxis) * spring->translationIAcc;
        real angularIB = spring->rB.cross(spring->tAxis) * spring->translationIAcc;

        vArray[bodyIDA] -= (spring->invMassA * linearI);
        wArray[bodyIDA] -= (spring->invMomentA * angularIA);

        vArray[bodyIDB] += (spring->invMassB * linearI);
        wArray[bodyIDB] += (spring->invMomentB * angularIB);
    }

    // rotation constraint
    {
        wArray[bodyIDB] += (spring->invMomentB * spring->rotationIAcc);
        wArray[bodyIDA] -= (spring->invMomentA * spring->rotationIAcc);
    }
}

void ftJointSolver::solveSpringJoint(ftJoint *joint,
                                     ftVector2 *vArray,
                                     real *wArray)
{

    ftSpringJoint *spring = (ftSpringJoint *)joint;
    int32 bodyIDA = spring->bodyIDA;
    int32 bodyIDB = spring->bodyIDB;

    //translation constraint
    {
        ftVector2 dv = vArray[bodyIDB] +
                       spring->rB.invCross(wArray[bodyIDB]) -
                       vArray[bodyIDA] -
                       spring->rA.invCross(wArray[bodyIDA]);
        real jv = dv.dot(spring->tAxis);

        real impulse = spring->invKTrans * (jv * -1);

        spring->translationIAcc += impulse;

        ftVector2 linearI = spring->tAxis * impulse;
        real angularIA = spring->rA.cross(spring->tAxis) * impulse;
        real angularIB = spring->rB.cross(spring->tAxis) * impulse;

        vArray[bodyIDA] -= (spring->invMassA * linearI);
        wArray[bodyIDA] -= (spring->invMomentA * angularIA);

        vArray[bodyIDB] += (spring->invMassB * linearI);
        wArray[bodyIDB] += (spring->invMomentB * angularIB);
    }

    //rotation constraint
    {
        real jv = wArray[bodyIDB] - wArray[bodyIDA];
        real impulse = spring->invKRot * (jv * -1);

        spring->rotationIAcc += impulse;

        wArray[bodyIDB] += spring->invMomentB * impulse;
        wArray[bodyIDA] -= spring->invMomentA * impulse;
    }
}
// End of spring joint