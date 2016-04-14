//
// Created by Kevin Yu on 2/9/16.
//

#ifndef FALTON_FTROTATION_H
#define FALTON_FTROTATION_H

#include "precision.h"
#include "falton/math/ftVector2.h"

class ftRotation {
public:
    real cosValue;
    real sinValue;
    real angle;

    ftRotation() : cosValue(0), sinValue(0) {}

    ftRotation(real angle) {
        this->angle = angle;
        cosValue = cos(angle);
        sinValue = sin(angle);
    }

    void setAngle(real angle) {
        this->angle = angle;
        cosValue = cos(angle);
        sinValue = sin(angle);
    }

    void operator+=(real angle);
    ftRotation operator*(const ftRotation& rhs) const;
    void operator*= (const ftRotation& rhs);
    ftVector2 operator* (const ftVector2& rhs) const;

};

inline void ftRotation::operator+=(real angle) {
    setAngle(this->angle + angle);
}

inline ftRotation ftRotation::operator*(const ftRotation &rhs) const {
    ftRotation rotation;
    rotation.cosValue = this->cosValue * rhs.cosValue - this->sinValue * rhs.sinValue;
    rotation.sinValue = this->sinValue * rhs.cosValue + this->cosValue * rhs.sinValue;

    return rotation;
}

inline void ftRotation::operator*=(const ftRotation &rhs) {

    real resultCosValue, resultSinValue;

    resultCosValue = this->cosValue * rhs.cosValue - this->sinValue * rhs.sinValue;
    resultSinValue = this->sinValue * rhs.cosValue + this->cosValue * rhs.sinValue;

    this->cosValue = resultCosValue;
    this->sinValue = resultSinValue;
}

inline ftVector2 ftRotation::operator*(const ftVector2& rhs) const {
    ftVector2 result;

    result.x = cosValue * rhs.x - sinValue * rhs.y;
    result.y = sinValue * rhs.x + cosValue * rhs.y;

    return result;
}

#endif //FALTON_FTROTATION_H
