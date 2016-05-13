//
// Created by Kevin Yu on 2016-05-12.
//

#ifndef FALTON_SETTINGS_H
#define FALTON_SETTINGS_H

#include <cassert>
#define ftAssert(X) assert(X)

//precision setting
#include <cmath>
#include <limits>

#define real float
#define real_sqrt sqrtf
#define PI M_PI
#define real_minInfinity -1 * std::numeric_limits<float>::infinity();
#define real_Infinity std::numeric_limits<float>::max();

//integer primitive setting
typedef char int8;
typedef short int16;
typedef long int32;
typedef long long int64;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
typedef unsigned long long uint64;

#endif //FALTON_SETTINGS_H
