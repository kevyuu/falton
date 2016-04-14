//
// Created by Kevin Yu on 12/1/15.
//

#ifndef FALTON_FTVECTOR2_H
#define FALTON_FTVECTOR2_H

#include "falton/math/precision.h"

struct ftVector2 {

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

    ftVector2 operator+(const ftVector2& v) const;
    ftVector2 operator-(const ftVector2& v) const;
    ftVector2 operator*(real scale) const;
    ftVector2 operator/(real scale) const;
    ftVector2 perpendicular() const;
    real cross(const ftVector2& v) const;
    ftVector2 invCross(real value) const;
    real dot(const ftVector2& v) const;

    real magnitude() const;
    real square_magnitude() const;
    ftVector2 unit() const;
    ftVector2 tangent() const;

    bool operator == (const ftVector2& v) const;

    void operator+=(const ftVector2& v);
    void operator-=(const ftVector2& v);
    void operator*=(real scale);
    void operator/=(real scale);
    void invert();
    void normalise();
    void setZero();
    void set(real x, real y);

};

inline ftVector2 ftVector2::operator+(const ftVector2 &v) const {
    return ftVector2(x + v.x, y + v.y);
}

inline ftVector2 ftVector2::operator-(const ftVector2 &v) const {
    return ftVector2(x - v.x, y - v.y);
}

inline ftVector2 ftVector2::operator*(real scale) const {
    return ftVector2(x * scale, y * scale);
}

inline ftVector2 operator* (real scale, const ftVector2& vector) {
    return ftVector2(vector.x * scale, vector.y * scale);
}

inline ftVector2 ftVector2::operator/(real scale) const{
    return ftVector2(x / scale, y / scale);
}

inline ftVector2 operator/ (real scale, const ftVector2& vector) {
    return ftVector2(vector.x / scale, vector.y / scale);
}

inline real ftVector2::cross(const ftVector2& v) const {
    return x * v.y - y * v.x;
}

//cross operation between vector (0,0,value) and this vector
inline ftVector2 ftVector2::invCross(real value) const {
    return ftVector2(value * -y, value * x);
}

inline real ftVector2::dot(const ftVector2& v) const {
    return x * v.x + y * v.y;
}

inline real ftVector2::magnitude() const {
    return real_sqrt(square_magnitude());
}

inline real ftVector2::square_magnitude() const {
    return x * x + y * y;
}

inline ftVector2 ftVector2::unit() const {
    return (*this) * (1/magnitude());
}

inline ftVector2 ftVector2::tangent() const {
    return ftVector2(y,-x);
}

inline ftVector2 ftVector2::perpendicular() const {
    return ftVector2(this->y, -1 * this->x);
}

inline bool ftVector2::operator==(const ftVector2& v) const {
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

inline void ftVector2::operator/=(real scale) {
    x /= scale;
    y /= scale;
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

inline void ftVector2::set(real x, real y) {
    this->x = x;
    this->y = y;
}

#endif //FALTON_FTVECTOR2_H