//
// Created by Kevin Yu on 12/12/15.
//

#ifndef FALTON_CONTACT_H
#define FALTON_CONTACT_H

#include "falton/math/math.h"

struct Contact {
    ftVector2 normal;
    real penetration_depth;
};


#endif //FALTON_CONTACT_H
