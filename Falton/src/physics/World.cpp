//
// Created by Kevin Yu on 10/9/15.
//

#include "falton/physics/world.h"

namespace falton {

    void World::startFrame() {
        BodyList *current = bodyList;

        while (current!= 0) {
            current->body->clear_accumulator();
            current = current->next;
        }
    }

    void World::step(real duration) {
        BodyList *current = bodyList;

        while (current!= 0) {
            current->body->integrate(duration);
            current = current->next;
        }

    }
}
