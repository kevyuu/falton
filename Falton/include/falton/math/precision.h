//
// Created by Kevin Yu on 9/2/15.
//

#ifndef FALTON_PRECISION_H
#define FALTON_PRECISION_H

#include "math.h"
#include <float.h>
#define real_sqrt sqrt
#define real_pow pow
#define real_abs fabs

namespace falton {
    typedef double real;
    const float EPSILON = DBL_EPSILON;

}

#endif //FALTON_PRECISION_H
