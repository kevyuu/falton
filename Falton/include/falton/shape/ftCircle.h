//
// Created by Kevin Yu on 12/12/15.
//

#ifndef FALTON_CIRCLE_H
#define FALTON_CIRCLE_H

#include <falton/math.h>
#include <falton/shape/ftShape.h>

class ftCircle : public ftShape{

public:
    real radius;
    real area;
    ftCircle();
    real getArea();
    void copy(const ftShape* shape);
    ftAABB constructAABB(ftTransform transform) const override;

    static ftCircle* create(real radius);
};


#endif //FALTON_CIRCLE_H
