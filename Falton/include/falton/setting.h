//
// Created by Kevin Yu on 2016-05-12.
//

#ifndef FALTON_SETTINGS_H
#define FALTON_SETTINGS_H

#include <cassert>
#include <cstdint>
#include <iostream>

//precision setting
#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <cassert>

#ifndef NDEBUG
#   define ftAssert(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::terminate(); \
        } \
    } while (false)
#else
#   define ftAssert(condition, message) do { } while (false)
#endif

#define real float
#define real_sqrt sqrtf
#define PI 3.14
#define real_minInfinity -1 * std::numeric_limits<float>::infinity();
#define real_Infinity std::numeric_limits<float>::max();
#define nulluint 0xFFFFFFFF

//integer primitive setting
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

#endif //FALTON_SETTINGS_H
