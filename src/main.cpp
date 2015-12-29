// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>

// Include falton
#include <falton/math/math.h>
#include <falton/physics/physics.h>
#include <falton/physics/shape/Shape.h>

using namespace std;

ftBodyTable bodyTable(128);
ftColliderTable colliderTable;
ftShapeTable shapeTable;

ftBodyHandle bodyHandle1;
ftBodyHandle bodyHandle2;

ftShapeHandle shapeHandle1;
ftShapeHandle shapeHandle2;

ftColliderHandle colHandle1;
ftColliderHandle colHandle2;

real gravity = 9.8f;

void step(real dt) {
    ftVector2 velocity = bodyTable.getVelocity(bodyHandle2);
    velocity.y += gravity * dt;
    bodyTable.setVelocity(bodyHandle2, velocity);

    ftVector2 position = bodyTable.getPosition(bodyHandle2);
    position += velocity * dt;
    bodyTable.setPosition(bodyHandle2, position);
}

int main( void )
{
    ftBodyDef bodyDef;
    bodyHandle1 = bodyTable.create(bodyDef);
    bodyDef.position = ftVector2(100,100);
    bodyHandle2 = bodyTable.create(bodyDef);

    shapeHandle1 = shapeTable.createBox(ftVector2(100,100));
    shapeHandle2 = shapeTable.createBox(ftVector2(10,10));

    ftColliderDef colliderDef;
    colliderDef.restitution = 0.2;
    colliderDef.friction = 0.3;

    colHandle1 = colliderTable.addCollider(bodyHandle1, colliderDef);
    colHandle2 = colliderTable.addCollider(bodyHandle2, colliderDef);

    return 0;
}
