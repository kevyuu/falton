//
// Created by Kevin Yu on 9/13/15.
//

#ifndef FALTON_MATRIX3X3_H
#define FALTON_MATRIX3X3_H

#include "precision.h"
#include "Quaternion.h"

namespace falton {
    class Matrix3x3 {

    public:

        real element[3][3];

        Matrix3x3();
        Matrix3x3(const Quaternion &rotationQuaternion);
        Matrix3x3(real e00,real e01, real e02,
                    real e10, real e11, real e12,
                    real e20, real e21, real e22);

        Matrix3x3 operator*(const Matrix3x3 &other) const;
        Vector3 operator*(const Vector3& vector) const;
        bool operator==(const Matrix3x3 &rhs) const;
        Matrix3x3 getInverse() const;
        Matrix3x3 getTranspose() const;
        real getDeterminant() const;

        void setOrientation(const Quaternion &orientation);
        void setInverse();
        void setTranspose();
        void setBlockInertiaTensor(const Vector3& halfSizes, real mass);

        Vector3 transform(const Vector3 &vector3) const;
        Vector3 transformInverse(const Vector3 &vector3) const;
        Vector3 transformTranspose(const Vector3 &vector3) const;

        void print();
        static Matrix3x3 linearInterpolate(const Matrix3x3& firstMatrix, const Matrix3x3& secondMatrix, real proportion);

    private:

    };
}


#endif //FALTON_MATRIX3X3_H
