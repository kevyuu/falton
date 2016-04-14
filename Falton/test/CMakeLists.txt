enable_testing()

set(GTEST_DIR "./gtest-1.7.0"
        CACHE PATH "The path to the GoogleTest test framework." FORCE)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # force this option to ON so that Google Test will use /MD instead of /MT
    # /MD is now the default for Visual Studio, so it should be our default, too
    option(gtest_force_shared_crt
            "Use shared (DLL) run-time lib even when Google Test is built as static lib."
            ON)
elseif (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=1)
endif()

add_subdirectory(${GTEST_DIR} ${CMAKE_BINARY_DIR}/gtest)
set_property(TARGET gtest APPEND_STRING PROPERTY COMPILE_FLAGS " -w")

include_directories(SYSTEM ${GTEST_DIR}/include)

add_executable(PolygonTest ftPolygonTest.cpp)
add_executable(CollisionTest ftCollisionTest.cpp ../include/Ball.cpp ../include/Ball.h)

target_link_libraries(PolygonTest falton gtest gtest_main)
target_link_libraries(CollisionTest falton gtest gtest_main)

add_test(PolygonTest PolygonTest)
add_test(CollisionTest CollisionTest)



