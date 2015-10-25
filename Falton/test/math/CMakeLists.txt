add_executable(Matrix3x3test Matrix3x3test.cpp)
target_link_libraries(Matrix3x3test gtest_main falton)

add_executable(QuaternionTest QuaternionTest.cpp)
target_link_libraries(QuaternionTest gtest_main falton)

add_test(Matrix3x3test Matrix3x3test)
add_test(QuaternionTest QuaternionTest)