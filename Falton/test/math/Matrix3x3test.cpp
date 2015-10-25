//
// Created by Kevin Yu on 10/11/15.
//
#include "falton/math/Matrix3x3.h"

#include <string>
using std::string;

#include <iostream>

#include <gtest/gtest.h>

namespace falton {

    class Matrix3x3Test : public testing::Test {

    public:

        const Matrix3x3 matrix1;
        const Matrix3x3 matrix2;
        const Vector3 vector1;
        const Quaternion quaternion1;

        Matrix3x3Test() :
                matrix1(
                5,3,4,
                3,6,7,
                10,11,12
            ), matrix2 (
                2,3,6,
                1,3,2,
                9,8,7
            ),vector1(3,5,7)
            ,quaternion1(0.7071067811865476,0,0.7071067811865476,0)
        {

        }
    };

    TEST_F (Matrix3x3Test, matrix_matrix_mutlipication) {

        Matrix3x3 expected(
                49,56,64,
                75,83,79,
                139,159,166
        );
        Matrix3x3 result = matrix1 * matrix2;

        EXPECT_EQ(expected, result);

    }

    TEST_F (Matrix3x3Test, matrix_vector_multiplication) {
        Vector3 expected(58,88,169);
        Vector3 result = matrix1 * vector1;
        Vector3 result2 = matrix1.transform(vector1);

        EXPECT_EQ(expected,result);
        EXPECT_EQ(expected,result2);
    }

    TEST_F (Matrix3x3Test, matrix_transpose) {
        Matrix3x3 expected(
                5,3,10,
                3,6,11,
                4,7,12
        );

        Matrix3x3 result = matrix1.getTranspose();
        Matrix3x3 copy = matrix1;
        EXPECT_EQ(copy,matrix1);

        copy.setTranspose();

        EXPECT_EQ(expected,result);
        EXPECT_EQ(expected,copy);

        EXPECT_EQ(result*vector1,matrix1.transformTranspose(vector1));
    }

    TEST_F (Matrix3x3Test, determinant) {
        real expected = -31;

        EXPECT_FLOAT_EQ(expected,matrix1.getDeterminant());
    }

    TEST_F (Matrix3x3Test, inverse) {
        Matrix3x3 expected(
                (real)5/31,(real)-8/31,(real)3/31,
                (real)-34/31,(real)-20/31,(real)23/31,
                (real)27/31,(real)25/31,(real)-21/31
        );

        Matrix3x3 result = matrix1.getInverse();
        Matrix3x3 copy = matrix1;
        EXPECT_EQ(copy,matrix1);

        copy.setInverse();
        EXPECT_EQ(expected,result);
        EXPECT_EQ(expected,copy);

        EXPECT_EQ(result*vector1,matrix1.transformInverse(vector1));

    }

    TEST_F (Matrix3x3Test, linear_interpolate) {
        Matrix3x3 expected(
                4.4, 3, 4.4,
                2.6, 5.4 , 6,
                9.8, 10.4, 11
        );
        Matrix3x3 result = Matrix3x3::linearInterpolate(matrix1,matrix2,0.2f);

        EXPECT_EQ(expected,result);
    }

    TEST_F (Matrix3x3Test, quaternion_conversion) {
        Matrix3x3 expected(
                0,0,-1,
                0,1,0,
                1,0,0
        );
        Matrix3x3 result(quaternion1);
        Matrix3x3 result2;
        result2.setOrientation(quaternion1);

        EXPECT_EQ(expected,result);
        EXPECT_EQ(expected,result2);
    }

    int main(int argc, char **argv) {
        testing::InitGoogleTest(&argc,argv);
        return RUN_ALL_TESTS();
    }
}
