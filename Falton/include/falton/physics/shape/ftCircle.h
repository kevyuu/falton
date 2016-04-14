//
// Created by Kevin Yu on 12/12/15.
//

#ifndef FALTON_CIRCLE_H
#define FALTON_CIRCLE_H

#include "falton/math/math.h"
#include "falton/physics/shape/ftShape.h"


class ftCircle : public ftShape{

public:
    real radius;
    real area;
    ftCircle(real radius);
    real getArea();
    real computeMomentInertia(real density);
    ftAABB constructAABB(ftTransform transform);
};


#endif //FALTON_CIRCLE_H
