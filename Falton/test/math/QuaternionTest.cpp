//
// Created by Kevin Yu on 10/13/15.
//

#include "falton/math/Quaternion.h"
#include <gtest/gtest.h>

namespace falton {

    class QuaternionTest : public testing::Test {
    public:
        const Quaternion quaternion1;
        const Quaternion quaternion2;
        const Quaternion quaternion3;
        const Quaternion quaternion4;

         QuaternionTest() :
            quaternion1(0.7071067811865476,0,0.7071067811865476,0)
            ,quaternion2(0.9980576390458148, 0.044490570256204946, -0.024614329527963587,
                         0.03599545919224397)
            ,quaternion3(3,4,5,6)
            ,quaternion4(1,3,5,7)
        {

        };

    };

    TEST_F(QuaternionTest, quaternion_quaternion_multiplication) {
        Quaternion expected(-76,18,10,32);

        EXPECT_EQ(expected, quaternion3 * quaternion4);

        Quaternion copy = quaternion3;
        copy *= quaternion4;
        EXPECT_EQ(expected, copy);
    }

    TEST_F (QuaternionTest, quaternion_scale_multiplication) {
        Quaternion expected(6,8,10,12);

        EXPECT_EQ(expected, quaternion3 * 2);

        Quaternion copy = quaternion3;
        copy*=2;
        EXPECT_EQ(expected, copy);
    }

    TEST_F (QuaternionTest, quaternion_addition) {
        Quaternion expected(4,7,10,13);

        EXPECT_EQ(expected, quaternion3 + quaternion4);

        Quaternion copy = quaternion3;
        copy += quaternion4;
        EXPECT_EQ(expected,copy);
    }

    TEST_F (QuaternionTest, quaternion_substraction) {
        Quaternion expected (2,1,0,-1);

        EXPECT_EQ(expected, quaternion3 - quaternion4);

        Quaternion copy = quaternion3;
        copy -= quaternion4;
        EXPECT_EQ(expected,copy);
    }

    TEST_F (QuaternionTest, inverse) {
        Quaternion expected(1,0,0,0);

        Quaternion inverse = quaternion1.inverse();
        EXPECT_EQ(expected,inverse*quaternion1);

        Quaternion copy = quaternion2;
        copy.invert();
        EXPECT_EQ(expected,copy * quaternion2);
    }

    TEST_F (QuaternionTest, magnitude) {

        EXPECT_FLOAT_EQ(86,quaternion3.squareMagnitude());
        EXPECT_FLOAT_EQ(9.273618495, quaternion3.magnitude());
    }

    TEST_F (QuaternionTest, conjugate) {
        Quaternion expected (3, -4,-5,-6);

        EXPECT_EQ(expected, quaternion3.conjugate());

        Quaternion copy = quaternion3;
        copy.setConjugate();
        EXPECT_EQ(expected,copy);
    }

    TEST_F (QuaternionTest, normalise) {
        EXPECT_FLOAT_EQ(1, quaternion3.norm().magnitude());

        Quaternion copy = quaternion3;
        copy.normalise();
        EXPECT_FLOAT_EQ(1, copy.magnitude());
    }

    int main(int argc, char **argv) {
        testing::InitGoogleTest(&argc,argv);
        return RUN_ALL_TESTS();
    }
}
