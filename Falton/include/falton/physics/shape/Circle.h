//
// Created by Kevin Yu on 12/12/15.
//

#ifndef FALTON_CIRCLE_H
#define FALTON_CIRCLE_H

#include "falton/math/math.h"

struct Circle {
    real radius;

    real computeMass(real density) {
        return 2 * PI * radius * radius;
    }
};


#endif //FALTON_CIRCLE_H
