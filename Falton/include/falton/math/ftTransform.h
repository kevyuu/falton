//
// Created by Kevin Yu on 12/14/15.
//

#ifndef FALTON_FTTRANSFORM_H
#define FALTON_FTTRANSFORM_H

#include "falton/math/precision.h"
#include "falton/math/ftVector2.h"
#include "falton/math/ftRotation.h"


class ftTransform {
public:
    ftVector2 center;

    ftRotation rotation;

    ftTransform() : center(0,0), rotation(0) {}

    ftTransform(ftVector2 center, real angle) : center(center), rotation(angle) {}

    ftVector2 operator*(const ftVector2& vec) const {
        ftVector2 result = rotation * vec;

        result += center;

        return result;
    }

    ftTransform operator*(const ftTransform& transform) const {
        ftTransform result;

        result.rotation = this->rotation * transform.rotation;

        result.center = this->rotation * transform.center;
        result.center += this->center;

        return result;
    }

    void operator*=(const ftTransform& transform) {

        this->center = (this->rotation * transform.center) + this->center;
        this->rotation *= transform.rotation;

    }
};


#endif //FALTON_FTTRANSFORM_H
