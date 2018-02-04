//
// Created by Kevin Yu on 4/10/16.
//
#pragma once

#include <falton/shape/ftCircle.h>
#include <falton/shape/ftPolygon.h>
#include <falton/math.h>

struct ftMassProperty {
    ftVector2 centerOfMass;
    real mass;
    real moment;
};

class ftMassComputer {
public:
	static ftMassProperty computeForCircle(real radius, real mass, ftVector2 offset);
	static ftMassProperty computeForPolygon(int vertexCount, ftVector2* vertexes, real mass, ftVector2 offset);

    static ftMassProperty computeForCircle(const ftCircle& circle, real mass, ftVector2 offset);
    static ftMassProperty computeForPolygon(const ftPolygon& polygon, real mass, ftVector2 offset);
};
