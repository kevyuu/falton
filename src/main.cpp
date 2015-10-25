// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
// Include GLEW
#include <GL/glew.h>
// Include GLFW
#include <GLFW/glfw3.h>

// Include falton
#include <falton/math.h>
#include <falton/physics.h>

#include <limits>
typedef std::numeric_limits< double > dbl;
GLFWwindow* window;

using namespace falton;
using namespace std;

int main( void )
{
    Vector3 windspeed(0,0,0);

    AeroControl right_wing(falton::Matrix3x3(0,0,0, -1,-0.5f,0, 0,0,0),
               falton::Matrix3x3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
               falton::Matrix3x3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
               falton::Vector3(-1.0f, 0.0f, 2.0f), &windspeed);

    AeroControl left_wing(falton::Matrix3x3(0,0,0, -1,-0.5f,0, 0,0,0),
                      falton::Matrix3x3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
                      falton::Matrix3x3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
                      falton::Vector3(-1.0f, 0.0f, -2.0f), &windspeed);

    AeroControl rudder(falton::Matrix3x3(0,0,0, 0,0,0, 0,0,0),
                   falton::Matrix3x3(0,0,0, 0,0,0, 0.01f,0,0),
                   falton::Matrix3x3(0,0,0, 0,0,0, -0.01f,0,0),
                   falton::Vector3(2.0f, 0.5f, 0), &windspeed);

    Aero tail(falton::Matrix3x3(0,0,0, -1,-0.5f,0, 0,0,-0.1f),
                 falton::Vector3(2.0f, 0, 0), &windspeed);


    real left_wing_control(-0.1), right_wing_control(0.1), rudder_control(0);

    right_wing.setControl(right_wing_control);
    left_wing.setControl(left_wing_control);
    rudder.setControl(rudder_control);

    RigidBody aircraft;
    aircraft.setOrientation(Quaternion(1,0,0,0));
    aircraft.setMass(2.5);
    aircraft.setLinearDamping(0.8f);
    aircraft.setAngularDamping(0.8f);
    Matrix3x3 it;
    it.setBlockInertiaTensor(Vector3(2,1,1), 1);
    aircraft.setInertiaTensor(it);

    ForceRegistry registry;
    Gravity gravity(Vector3(0,-9.81*2.5,0));
    registry.add(&aircraft,&gravity);
    registry.add(&aircraft,&right_wing);
    registry.add(&aircraft,&left_wing);
    registry.add(&aircraft,&rudder);
    registry.add(&aircraft,&tail);

    int numberOfIteration;
    cout<<"Number of Iteration :";
    cin>>numberOfIteration;
    while(numberOfIteration!=0) {
        for (int i=0;i<numberOfIteration;i++){

            aircraft.clear_accumulator();

            Vector3 propulsion = aircraft.localToWorldDirection(Vector3(-10,0,0));
            aircraft.applyForceToCenterOfMass(propulsion);
            registry.applyForce(0.016);
            aircraft.integrate(0.016);

            falton::Vector3 pos = aircraft.getPosition();
            if (pos.y < 0.0f)
            {
                pos.y = 0.0f;
                aircraft.setPosition(pos);

                if (aircraft.getLinearVelocity().y < -10.0f)
                {
                    aircraft.setPosition(falton::Vector3(0,0,0));
                    aircraft.setOrientation(falton::Quaternion(1,0,0,0));

                    aircraft.setLinearVelocity(falton::Vector3(0,0,0));
                    aircraft.setAngularVelocity(falton::Vector3(0,0,0));
                }
            }

        }

        Vector3 pos = aircraft.getPosition();
        cout.precision(dbl::max_digits10);
        std::cout<<pos.x<<" "<<pos.y<<" "<<pos.z<<std::endl;

        Quaternion orientation = aircraft.getOrientation();
        cout.precision(dbl::max_digits10);
        cout<<orientation.scalar()<<" "<<orientation.vector().x<<" "<<orientation.vector().y<<" "<<orientation.vector().z<<endl;

        cout<<"Number of Iteration :";
        cin>>numberOfIteration;
    }

    return 0;
}
