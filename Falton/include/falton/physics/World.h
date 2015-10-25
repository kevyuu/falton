//
// Created by Kevin Yu on 10/9/15.
//

#ifndef TUTORIAL_WORLD_H
#define TUTORIAL_WORLD_H

#include "falton/math.h"
#include "falton/physics/RigidBody.h"
namespace falton {
    class World {
    public:

        void startFrame();

        void step(real duration);

        void addBody(RigidBody *body);

    private :
        struct BodyList {
            RigidBody *body;
            BodyList *next;
        };

        BodyList *bodyList;


    };
}



#endif //TUTORIAL_WORLD_H
