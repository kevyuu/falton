//
// Created by Kevin Yu on 12/14/15.
//

#ifndef FALTON_FTTRANSFORM_H
#define FALTON_FTTRANSFORM_H

#include "falton/math/precision.h"
#include "falton/math/ftVector2.h"

class ftTransform {
public:
    ftVector2 center;

    real cosAngle;
    real sinAngle;

    ftVector2 operator*(const ftVector2& vec) {
        ftVector2 result(0,0);
        result.x = cosAngle * vec.x - sinAngle * vec.y + center.x;
        result.y = sinAngle * vec.y + cosAngle * vec.x + center.y;

        return result;
    }

};


#endif //FALTON_FTTRANSFORM_H
