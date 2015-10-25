//
// Created by Kevin Yu on 9/21/15.
//

#ifndef FALTON_QUATERNION_H
#define FALTON_QUATERNION_H

#include "precision.h"
#include "Vector3.h"

namespace falton {
    class Quaternion {

    public:

        Quaternion();
        Quaternion(const Vector3 &axis, real angle);
        Quaternion(const Vector3 &orientation);
        Quaternion(real w, real i, real j, real k);
        Quaternion(real w, Vector3 v);

        Quaternion operator*(const Quaternion &rhs) const;
        Quaternion operator*(real scale) const;
        Quaternion operator+(const Quaternion &rhs) const;
        Quaternion operator-(const Quaternion &rhs) const;
        Quaternion inverse() const;
        Quaternion conjugate() const;
        Quaternion norm() const;
        bool operator==(const Quaternion &rhs) const;


        Quaternion& operator*=(const Quaternion &rhs);
        Quaternion& operator*=(real scale);
        Quaternion& operator+=(const Quaternion &rhs);
        Quaternion& operator-=(const Quaternion &rhs);
        void invert();
        void setConjugate();
        void normalise();
        void addScaledVector(const Vector3 &vec,real scale);

        real squareMagnitude() const;
        real magnitude() const;

        real scalar() const;
        Vector3 vector() const;
        Vector3 transform(const Vector3 &vector) const;
        Vector3 transformInverse(const Vector3 &vector) const;


    private:
        real w;
        Vector3 v;
    };

}


#endif //FALTON_QUATERNION_H
