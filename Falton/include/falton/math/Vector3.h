//
// Created by Kevin Yu on 9/20/15.
//

#ifndef FALTON_VECTOR3_H
#define FALTON_VECTOR3_H

#include "precision.h"

namespace falton {

    class Vector3 {
    public:
        real x;
        real y;
        real z;

        Vector3();

        Vector3(real x,real y,real z);

        real dot(const Vector3 &rhs) const;
        Vector3 cross(const Vector3 &rhs) const;
        Vector3 operator+(const Vector3 &rhs) const;
        Vector3 operator-(const Vector3 &rhs) const;
        Vector3 operator*(real scale) const;
        bool operator==(const Vector3 &rhs) const;

        void cross_update(const Vector3 &rhs);
        void operator*=(const Vector3 &rhs);
        Vector3& operator+=(const Vector3 &rhs);
        void operator-=(const Vector3 &rhs);
        void operator*=(real scale);
        void zero();
        void normalize();

        real magnitude() const;
        real square_magnitude() const;

    };

}


#endif //FALTON_VECTOR3_H
