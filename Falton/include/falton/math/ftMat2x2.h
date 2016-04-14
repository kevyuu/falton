//
// Created by Kevin Yu on 4/13/16.
//

#ifndef FALTON_FTMAT2X2_H
#define FALTON_FTMAT2X2_H

#include "falton/math/precision.h"

class ftMat2x2 {
public:
    real element[2][2];
    void invert();
    ftVector2 operator*(const ftVector2& rhs);
};

inline void ftMat2x2::invert() {
    real determinant = ((element[0][0] * element[1][1]) - (element[0][1] * element[1][0]));

    real tmp = element[0][0];
    element[0][0] = element[1][1] / determinant;
    element[1][1] = tmp / determinant;

    tmp = element[0][1];
    element[0][1] = - element[1][0] / determinant;
    element[1][0] = - tmp / determinant;
}

inline ftVector2 ftMat2x2::operator*(const ftVector2& rhs) {
    ftVector2 result;

    result.x = element[0][0] * rhs.x + element[0][1] * rhs.y;
    result.y = element[1][0] * rhs.x + element[1][1] * rhs.y;

    return result;
}

#endif //FALTON_FTMAT2X2_H
