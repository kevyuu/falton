//
// Created by Kevin Yu on 12/5/15.
//

#ifndef FALTON_MATH_H
#define FALTON_MATH_H

#include "ftVector2.h"
#include "precision.h"
#include "type.h"
#include "ftTransform.h"
#include "ftMat2x2.h"

inline real ftMin(real x, real y) {
    return x < y ? x : y;
}

inline real ftMax(real x, real y) {
    return x > y ? x : y;
}
#endif //FALTON_MATH_H
