//
// Created by Kevin Yu on 9/21/15.
//

#include "falton/math/Quaternion.h"

namespace falton{
    Quaternion::Quaternion():w(0), v() { }

    Quaternion::Quaternion(const Vector3 &axis, real angle) {

        real halfAngleInDegree = (M_PI * angle) / 360;
        w = cos(halfAngleInDegree);

        real scale = sin(halfAngleInDegree);

        v = axis;
        v *= scale;
    }

    Quaternion::Quaternion(const Vector3 &orientation) {
        real angleInRadian = orientation.magnitude();
        if (angleInRadian > 0) {
            real halfAngleInRadian = angleInRadian/2;
            w = cos(halfAngleInRadian);

            real scale = sin(halfAngleInRadian);

            v = orientation;
            v.normalize();
            v *= scale;

            normalise();
        } else {
            w = 1;
            v.zero();
        }
    }

    Quaternion::Quaternion(real w, real i, real j, real k) : w(w), v(i,j,k) { }

    Quaternion::Quaternion(real w, Vector3 v) : w(w), v(v) {}

    Quaternion Quaternion::operator*(const Quaternion &rhs) const {
        return Quaternion(w * rhs.w - v.dot(rhs.v) ,
                          (v * rhs.w) + (rhs.v * w) + v.cross(rhs.v));
    }

    Quaternion Quaternion::operator*(real scale) const {
        return Quaternion(w * scale, v * scale);
    }

    Quaternion Quaternion::operator+(const Quaternion &rhs) const {
        return Quaternion(w + rhs.w, v + rhs.v);
    }

    Quaternion Quaternion::operator-(const Quaternion &rhs) const {
        return Quaternion(w - rhs.w, v - rhs.v);
    }

    Quaternion Quaternion::inverse() const {
        Quaternion result (w, v * -1);
        result *= ((real)1/ squareMagnitude());
        return result;
    }

    Quaternion Quaternion::conjugate() const {
        return Quaternion(w, v * -1);
    }

    Quaternion Quaternion::norm() const {
        return Quaternion(w, v) * ((real) 1 / magnitude());
    }

    bool Quaternion::operator==(const Quaternion &rhs) const {

        if (real_abs(w-rhs.w)>EPSILON) return false;
        if (real_abs(v.x - rhs.v.x) > EPSILON) return false;
        if (real_abs(v.y - rhs.v.y) > EPSILON) return false;
        if (real_abs(v.z - rhs.v.z) > EPSILON) return false;
        return true;
    }

    Quaternion& Quaternion::operator*=(const Quaternion &rhs) {
        real wNew = w * rhs.w - v.dot(rhs.v);
        v = (v * rhs.w + rhs.v * w + v.cross(rhs.v));
        w = wNew;
        return (*this);
    }

    Quaternion& Quaternion::operator*=(real scale) {
        w *= scale;
        v *= scale;
        return (*this);
    }

    Quaternion& Quaternion::operator+=(const Quaternion &rhs) {
        w += rhs.w;
        v += rhs.v;
        return (*this);
    }

    Quaternion& Quaternion::operator-=(const Quaternion &rhs) {
        w -= rhs.w;
        v -= rhs.v;
        return (*this);
    }

    void Quaternion::invert() {
        float scale = (real)1 / squareMagnitude();
        w *= scale;
        v *= -scale;
    }

    void Quaternion::setConjugate() {
        v *= -1;
    }

    void Quaternion::normalise() {
        if (squareMagnitude() < EPSILON) {
            w = 1;
            return;
        }

        real scale = (real) 1 / magnitude();
        w *= scale;
        v *= scale;
    }

    real Quaternion::squareMagnitude() const{
        return w * w + v.square_magnitude();
    }

    real Quaternion::magnitude() const {
        real ans = real_sqrt(w * w + v.square_magnitude());
        return ans;
    }

    real Quaternion::scalar() const {
        return w;
    }

    Vector3 Quaternion::vector() const {
        return v;
    }

    Vector3 Quaternion::transform(const Vector3 &vector3) const {
        return ((*this) * Quaternion(0,vector3) * (*this).inverse()).v;
    }

    Vector3 Quaternion::transformInverse(const Vector3 &vector) const {
        return ((*this).inverse() * Quaternion(0,vector) * (*this)).v;
    }

    void Quaternion::addScaledVector(const Vector3 &vec, real scale) {
        Quaternion q (0, vec.x * scale,vec.y * scale,vec.z * scale);
        q *= (*this);
        w += q.w * (real(0.5));
        v += q.v * (real(0.5));
    }

}