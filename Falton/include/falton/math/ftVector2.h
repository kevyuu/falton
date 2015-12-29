//
// Created by Kevin Yu on 12/1/15.
//

#ifndef FALTON_FTVECTOR2_H
#define FALTON_FTVECTOR2_H

#include "falton/math/precision.h"

class ftVector2 {

public:
    real x;
    real y;

    ftVector2() {
        this->x = 0;
        this->y = 0;
    }

    ftVector2(real x, real y) {
        this->x = x;
        this->y = y;
    }

    ftVector2 operator+(const ftVector2& v);
    ftVector2 operator-(const ftVector2& v);
    ftVector2 operator*(real scale);
    real cross(ftVector2 v);
    real dot(ftVector2 v);

    real magnitude();
    real square_magnitude();
    ftVector2 unit();

    bool operator == (const ftVector2& v);

    void operator+=(const ftVector2& v);
    void operator-=(const ftVector2& v);
    void operator*=(real scale);
    void invert();
    void normalise();
    void setZero();

};

inline ftVector2 ftVector2::operator+(const ftVector2 &v) {
    return ftVector2(x + v.x, y + v.y);
}

inline ftVector2 ftVector2::operator-(const ftVector2 &v) {
    return ftVector2(x - v.x, y - v.y);
}

inline ftVector2 ftVector2::operator*(real scale) {
    return ftVector2(x * scale, y * scale);
}

inline real ftVector2::cross(ftVector2 v) {
    return x * v.y - y * v.x;
}

inline real ftVector2::magnitude() {
    return real_sqrt(square_magnitude());
}

inline real ftVector2::square_magnitude() {
    return x * x + y * y;
}

inline ftVector2 ftVector2::unit() {
    return (*this) * (1/magnitude());
}

inline bool ftVector2::operator==(const ftVector2& v) {
    return x == v.x && y == v.y;
}

inline void ftVector2::operator+=(const ftVector2& v) {
    x += v.x;
    y += v.y;
}

inline void ftVector2::operator-=(const ftVector2& v) {
    x -= v.x;
    y -= v.y;
}

inline void ftVector2::operator*=(real scale) {
    x *= scale;
    y *= scale;
}

inline void ftVector2::invert() {
    x *= -1;
    y *= -1;
}

inline void ftVector2::normalise() {
    real m = magnitude();
    x = x/m;
    y = y/m;
}

inline void ftVector2::setZero() {
    x = 0;
    y = 0;
}

#endif //FALTON_FTVECTOR2_H
