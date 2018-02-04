//
// Created by Kevin Yu on 12/5/15.
//

#ifndef FALTON_MATH_H
#define FALTON_MATH_H

#include <cstdlib>
#include <cmath>
#include <falton/math.h>
#include <falton/setting.h>

#define ftAbs abs
#define ftCeil ceil
#define ftFloor floor

inline int32 ftPositiveMod(int32 a, int32 b) {
    return (a % b + b) % b;
}

inline real ftMin(real x, real y) {
    return x < y ? x : y;
}

inline real ftMax(real x, real y) {
    return x > y ? x : y;
}

inline real ftClamp(real value, real min, real max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

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

class ftRotation {
public:
    real cosValue;
    real sinValue;
    real angle; // in radian

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

    static ftRotation Angle(real angle);
    static ftRotation Direction(ftVector2 direction);

    void operator+=(real angle);
    ftRotation operator*(const ftRotation& rhs) const;
    void operator*= (const ftRotation& rhs);
    ftVector2 operator* (const ftVector2& rhs) const;
    ftVector2 invRotate(const ftVector2 &rhs) const;

};

inline void ftRotation::operator+=(real angle) {
    setAngle(this->angle + angle);
}

inline ftRotation ftRotation::operator*(const ftRotation &rhs) const {
    ftRotation rotation;
    rotation.cosValue = this->cosValue * rhs.cosValue - this->sinValue * rhs.sinValue;
    rotation.sinValue = this->sinValue * rhs.cosValue + this->cosValue * rhs.sinValue;
    rotation.angle = this->angle + rhs.angle;

    return rotation;
}

inline void ftRotation::operator*=(const ftRotation &rhs) {

    real resultCosValue, resultSinValue;

    resultCosValue = this->cosValue * rhs.cosValue - this->sinValue * rhs.sinValue;
    resultSinValue = this->sinValue * rhs.cosValue + this->cosValue * rhs.sinValue;

    this->cosValue = resultCosValue;
    this->sinValue = resultSinValue;
    this->angle += rhs.angle;
}

inline ftVector2 ftRotation::operator*(const ftVector2& rhs) const {
    ftVector2 result;

    result.x = cosValue * rhs.x - sinValue * rhs.y;
    result.y = sinValue * rhs.x + cosValue * rhs.y;

    return result;
}

inline ftVector2 ftRotation::invRotate(const ftVector2 &rhs) const {
    ftVector2 result;

    result.x = cosValue * rhs.x + sinValue * rhs.y;
    result.y = -sinValue * rhs.x + cosValue * rhs.y;

    return result;
}

inline ftRotation ftRotation::Angle(real angle) {
    ftRotation rotation;
    rotation.cosValue = cos(angle);
    rotation.sinValue = sin(angle);
    rotation.angle = angle;
    return rotation;
}

inline ftRotation ftRotation::Direction(ftVector2 direction) {
    ftRotation rotation;
    real distance = sqrt(direction.x * direction.x + direction.y * direction.y);
    rotation.sinValue = direction.y / distance;
    rotation.cosValue = direction.x / distance;
    rotation.angle = atan2(direction.y, direction.x);
    return rotation;
}

class ftTransform {
public:
    ftVector2 center;

    ftRotation rotation;

    ftTransform() : center(0,0), rotation(0) {}

    ftTransform(ftVector2 center, ftRotation rotation) {
        this->center = center;
        this->rotation = rotation;
    }
    
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

    ftVector2 invTransform(const ftVector2 vector) {
        ftVector2 result = vector - center;
        return rotation.invRotate(result);
    }
};

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

    element[0][1] = - element[0][1] / determinant;
    element[1][0] = - element[1][0] / determinant;
}

inline ftVector2 ftMat2x2::operator*(const ftVector2& rhs) {
ftVector2 result;

    result.x = element[0][0] * rhs.x + element[1][0] * rhs.y;
    result.y = element[0][1] * rhs.x + element[1][1] * rhs.y;

    return result;
}

#endif //FALTON_MATH_H
