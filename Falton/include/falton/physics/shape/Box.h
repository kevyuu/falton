//
// Created by Kevin Yu on 12/12/15.
//

#ifndef FALTON_BOX_H
#define FALTON_BOX_H

#include "falton/math/math.h"

struct Box {

    Box(ftVector2 halfWidth) : half_width(halfWidth) {}
    ftVector2 half_width;

    real computeMass(real density) {
        return 4 * half_width.x * half_width.y;
    }

};


#endif //FALTON_BOX_H
