//
// Created by Kevin Yu on 9/13/15.
//

#include "falton/math/Matrix3x3.h"
#include <iostream>
using namespace std;

namespace falton {

    Matrix3x3::Matrix3x3() {
        for (int i=0;i<3;i++) {
            for (int j=0;j<3;j++) {
                element[i][j] = 0;
            }
        }
    }

    Matrix3x3::Matrix3x3(real e00, real e01, real e02,
                         real e10, real e11, real e12,
                         real e20, real e21, real e22) {
        element[0][0] = e00;
        element[0][1] = e01;
        element[0][2] = e02;

        element[1][0] = e10;
        element[1][1] = e11;
        element[1][2] = e12;

        element[2][0] = e20;
        element[2][1] = e21;
        element[2][2] = e22;

    }

    Matrix3x3::Matrix3x3(const Quaternion &rotationQuaternion) {

        real s = rotationQuaternion.scalar();
        Vector3 vector3 = rotationQuaternion.vector();

        element[0][0] = 1 - 2 * (vector3.y * vector3.y + vector3.z * vector3.z);
        element[0][1] = 2 * (vector3.x * vector3.y - s * vector3.z);
        element[0][2] = 2 * (vector3.x * vector3.z + s * vector3.y);

        element[1][0] = 2 * (vector3.x * vector3.y + s * vector3.z);
        element[1][1] = 1 - 2 * (vector3.x * vector3.x + vector3.z * vector3.z);
        element[1][2] = 2 * (vector3.y * vector3.z - s * vector3.x);

        element[2][0] = 2 * (vector3.x * vector3.z - s * vector3.y);
        element[2][1] = 2 * (vector3.y * vector3.z + s * vector3.x);
        element[2][2] = 1 - 2 * (vector3.x * vector3.x + vector3.y * vector3.y);

    }

    Matrix3x3 Matrix3x3::operator* (const Matrix3x3 &rhs) const {
        Matrix3x3 result_matrix;

        result_matrix.element[0][0] = element[0][0] * rhs.element[0][0] +
                                      element[0][1] * rhs.element[1][0] +
                                      element[0][2] * rhs.element[2][0];

        result_matrix.element[0][1] = element[0][0] * rhs.element[0][1] +
                                      element[0][1] * rhs.element[1][1] +
                                      element[0][2] * rhs.element[2][1];

        result_matrix.element[0][2] = element[0][0] * rhs.element[0][2] +
                                      element[0][1] * rhs.element[1][2] +
                                      element[0][2] * rhs.element[2][2];

        result_matrix.element[1][0] = element[1][0] * rhs.element[0][0] +
                                      element[1][1] * rhs.element[1][0] +
                                      element[1][2] * rhs.element[2][0];

        result_matrix.element[1][1] = element[1][0] * rhs.element[0][1] +
                                      element[1][1] * rhs.element[1][1] +
                                      element[1][2] * rhs.element[2][1];

        result_matrix.element[1][2] = element[1][0] * rhs.element[0][2] +
                                      element[1][1] * rhs.element[1][2] +
                                      element[1][2] * rhs.element[2][2];

        result_matrix.element[2][0] = element[2][0] * rhs.element[0][0] +
                                      element[2][1] * rhs.element[1][0] +
                                      element[2][2] * rhs.element[2][0];

        result_matrix.element[2][1] = element[2][0] * rhs.element[0][1] +
                                      element[2][1] * rhs.element[1][1] +
                                      element[2][2] * rhs.element[2][1];

        result_matrix.element[2][2] = element[2][0] * rhs.element[0][2] +
                                      element[2][1] * rhs.element[1][2] +
                                      element[2][2] * rhs.element[2][2];

        return result_matrix;
    }

    Vector3 Matrix3x3::operator*(const Vector3 &vector) const {
        return Vector3(
                element[0][0] * vector.x + element[0][1] * vector.y + element[0][2] * vector.z,
                element[1][0] * vector.x + element[1][1] * vector.y + element[1][2] * vector.z,
                element[2][0] * vector.x + element[2][1] * vector.y + element[2][2] * vector.z
        );
    }

    bool Matrix3x3::operator==(const Matrix3x3 &rhs) const {
        for (int i=0;i<3;i++) {
            for (int j=0;j<3;j++) {
                if (real_abs(element[i][j]-rhs.element[i][j])>EPSILON) return false;
            }
        }
        return true;
    }

    Matrix3x3 Matrix3x3::getInverse() const {
        real determinant = element[0][0] * (element[1][1] * element[2][2] - element[1][2] * element[2][1])
                           - element[0][1] * (element[1][0] * element[2][2] - element[1][2] * element[2][0])
                           + element[0][2] * (element[1][0] * element[2][1] - element[1][1] * element[2][0]);

        real inverseDet = 1/determinant;

        Matrix3x3 result;

        result.element[0][0] = inverseDet * (element[1][1] * element[2][2] - element[2][1] * element[1][2]);
        result.element[0][1] = inverseDet * (element[2][1] * element[0][2] - element[0][1] * element[2][2]);
        result.element[0][2] = inverseDet * (element[0][1] * element[1][2] - element[1][1] * element[0][2]);
        result.element[1][0] = inverseDet * (element[2][0] * element[1][2] - element[1][0] * element[2][2]);
        result.element[1][1] = inverseDet * (element[0][0] * element[2][2] - element[2][0] * element[0][2]);
        result.element[1][2] = inverseDet * (element[1][0] * element[0][2] - element[0][0] * element[1][2]);
        result.element[2][0] = inverseDet * (element[1][0] * element[2][1] - element[2][0] * element[1][1]);
        result.element[2][1] = inverseDet * (element[2][0] * element[0][1] - element[0][0] * element[2][1]);
        result.element[2][2] = inverseDet * (element[0][0] * element[1][1] - element[0][1] * element[1][0]);

        return result;
    }

    Matrix3x3 Matrix3x3::getTranspose() const {
        Matrix3x3 result;

        for (int i=0;i<3;i++) {
            for (int j=0;j<3;j++) {
                result.element[i][j] = element[j][i];
            }
        }

        return result;
    }

    real Matrix3x3::getDeterminant() const {
        real determinant = element[0][0] * (element[1][1] * element[2][2] - element[1][2] * element[2][1])
                           - element[0][1] * (element[1][0] * element[2][2] - element[1][2] * element[2][0])
                           + element[0][2] * (element[1][0] * element[2][1] - element[1][1] * element[2][0]);
        return determinant;
    }

    void Matrix3x3::setOrientation(const Quaternion &rotationQuaternion) {
        real s = rotationQuaternion.scalar();
        Vector3 vector3 = rotationQuaternion.vector();

        element[0][0] = 1 - 2 * (vector3.y * vector3.y + vector3.z * vector3.z);
        element[0][1] = 2 * (vector3.x * vector3.y - s * vector3.z);
        element[0][2] = 2 * (vector3.x * vector3.z + s * vector3.y);

        element[1][0] = 2 * (vector3.x * vector3.y + s * vector3.z);
        element[1][1] = 1 - 2 * (vector3.x * vector3.x + vector3.z * vector3.z);
        element[1][2] = 2 * (vector3.y * vector3.z - s * vector3.x);

        element[2][0] = 2 * (vector3.x * vector3.z - s * vector3.y);
        element[2][1] = 2 * (vector3.y * vector3.z + s * vector3.x);
        element[2][2] = 1 - 2 * (vector3.x * vector3.x + vector3.y * vector3.y);

    }

    void Matrix3x3::setInverse() {
        real determinant = element[0][0] * (element[1][1] * element[2][2] - element[1][2] * element[2][1])
                           - element[0][1] * (element[1][0] * element[2][2] - element[1][2] * element[2][0])
                           + element[0][2] * (element[1][0] * element[2][1] - element[1][1] * element[2][0]);

        real inverseDet = 1/determinant;

        float cofactor00 = element[1][1] * element[2][2] - element[2][1] * element[1][2];
        float cofactor01 = element[2][1] * element[0][2] - element[0][1] * element[2][2];
        float cofactor02 = element[0][1] * element[1][2] - element[1][1] * element[0][2];
        float cofactor10 = element[2][0] * element[1][2] - element[1][0] * element[2][2];
        float cofactor11 = element[0][0] * element[2][2] - element[2][0] * element[0][2];
        float cofactor12 = element[1][0] * element[0][2] - element[0][0] * element[1][2];
        float cofactor20 = element[1][0] * element[2][1] - element[2][0] * element[1][1];
        float cofactor21 = element[2][0] * element[0][1] - element[0][0] * element[2][1];
        float cofactor22 = element[0][0] * element[1][1] - element[0][1] * element[1][0];

        element[0][0] = inverseDet * cofactor00;
        element[0][1] = inverseDet * cofactor01;
        element[0][2] = inverseDet * cofactor02;
        element[1][0] = inverseDet * cofactor10;
        element[1][1] = inverseDet * cofactor11;
        element[1][2] = inverseDet * cofactor12;
        element[2][0] = inverseDet * cofactor20;
        element[2][1] = inverseDet * cofactor21;
        element[2][2] = inverseDet * cofactor22;

    }

    void Matrix3x3::setTranspose() {

        real tmp = element[0][1];
        element[0][1] = element[1][0];
        element[1][0] = tmp;

        tmp = element[0][2];
        element[0][2] = element[2][0];
        element[2][0] = tmp;

        tmp = element[1][2];
        element[1][2] = element[2][1];
        element[2][1] = tmp;

    }

    void Matrix3x3::setBlockInertiaTensor(const Vector3 &halfSizes, real mass) {

        real squareX = halfSizes.x * halfSizes.x;
        real squareY = halfSizes.y * halfSizes.y;
        real squareZ = halfSizes.z * halfSizes.z;

        //zero all non diagonal value
        element[0][1] = element[1][0] = element[0][2] = element[2][0] = element[1][2] = element[2][1] = 0;

        // set diagonal value
        element[0][0] = (0.3f * mass * (squareY + squareZ));
        element[1][1] = (0.3f * mass * (squareX + squareZ));
        element[2][2] = (0.3f * mass * (squareX + squareY));

    }

    Vector3 Matrix3x3::transform(const Vector3 &vector) const {
        return Vector3(
                element[0][0] * vector.x + element[0][1] * vector.y + element[0][2] * vector.z,
                element[1][0] * vector.x + element[1][1] * vector.y + element[1][2] * vector.z,
                element[2][0] * vector.x + element[2][1] * vector.y + element[2][2] * vector.z
        );
    }

    Vector3 Matrix3x3::transformInverse(const Vector3 &vector) const {
        return getInverse().transform(vector);
    }

    Vector3 Matrix3x3::transformTranspose(const Vector3 &vector) const {
        return Vector3(
                element[0][0] * vector.x + element[1][0] * vector.y + element[2][0] * vector.z,
                element[0][1] * vector.x + element[1][1] * vector.y + element[2][1] * vector.z,
                element[0][2] * vector.x + element[1][2] * vector.y + element[2][2] * vector.z
        );
    }

    Matrix3x3 Matrix3x3::linearInterpolate(const Matrix3x3 &firstMatrix, const Matrix3x3 &secondMatrix, real proportion) {
        Matrix3x3 result;
        for (int i=0;i<3;i++) {
            for (int j=0;j<3;j++) {
                result.element[i][j] = firstMatrix.element[i][j] * (1-proportion) + secondMatrix.element[i][j] * proportion;
            }
        }
        return result;
    }

    void Matrix3x3::print() {
        for (int i=0;i<3;i++) {
            for (int j=0;j<3;j++) {
                std::cout<<element[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
    }
}
