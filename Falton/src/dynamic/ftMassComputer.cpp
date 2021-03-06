//
// Created by Kevin Yu on 4/10/16.
//

#include <falton/dynamic/ftMassComputer.h>

ftMassProperty ftMassComputer::computeForCircle(real radius, real mass, ftVector2 offset) {
	ftMassProperty mp;
	mp.mass = mass;
	mp.moment = mass * ((radius * radius / 2) + offset.square_magnitude());
	mp.centerOfMass.x = 0;
	mp.centerOfMass.y = 0;

	return mp;
}

ftMassProperty ftMassComputer::computeForPolygon(int vertexCount, ftVector2* vertexes, 
	real mass, ftVector2 offset) {
	ftMassProperty mp;
	mp.mass = mass;


	mp.moment = 0;
	for (int i = 0; i < vertexCount; ++i) {
		int i1 = i;
		int i2 = (i + 1) == vertexCount ? 0 : (i + 1);

		real x1 = vertexes[i1].x + offset.x;
		real y1 = vertexes[i1].y + offset.y;
		real x2 = vertexes[i2].x + offset.x;
		real y2 = vertexes[i2].y + offset.y;

		real cross = x1 * y2 - x2 * y1;

		mp.moment += ((x1 * x1 + x1 * x2 + x2 * x2 + y1 * y1 + y1 * y2 + y2 * y2) * cross);
		mp.centerOfMass.x += ((x1 + x2) * cross);
		mp.centerOfMass.y += ((y1 + y2) * cross);

	}

	real area = 0;
	for (int32 i = 0; i < vertexCount - 1; i++) {

		ftVector2 a = vertexes[i];
		ftVector2 b = vertexes[i + 1];

		area += a.cross(b) / 2;

	}
	area += ((vertexes[vertexCount - 1]).cross(vertexes[0]) / 2);

	real density = mass / area;
	mp.moment = (mp.moment * density) / 12;
	mp.centerOfMass /= (6 * area);

	return mp;
}

ftMassProperty ftMassComputer::computeForCircle(const ftCircle &circle, real mass, ftVector2 offset) {

    ftMassProperty mp;
    mp.mass = mass;
    mp.moment = mass  * ((circle.radius * circle.radius / 2) + offset.square_magnitude());
    mp.centerOfMass.x = 0;
    mp.centerOfMass.y = 0;

    return mp;
}

ftMassProperty ftMassComputer::computeForPolygon(const ftPolygon &polygon, real mass, ftVector2 offset) {

    ftMassProperty mp;
    mp.mass = mass;

    uint8 numVertex = polygon.numVertex;
    ftVector2 *vertices = polygon.vertices;

    mp.moment = 0;
    for (int i=0;i<numVertex;++i) {
        int i1 = i;
        int i2 = (i+1) == numVertex ? 0 : (i+1);

        real x1 = vertices[i1].x + offset.x;
        real y1 = vertices[i1].y + offset.y;
        real x2 = vertices[i2].x + offset.x;
        real y2 = vertices[i2].y + offset.y;

        real cross = x1 * y2 - x2 * y1;

        mp.moment += ((x1 * x1 + x1 * x2 + x2 * x2 + y1 * y1 + y1 * y2 + y2 * y2) * cross);
        mp.centerOfMass.x += ((x1 + x2) * cross);
        mp.centerOfMass.y += ((y1 + y2) * cross);

    }

    real density = mass/polygon.area;
    mp.moment =  (mp.moment * density )/ 12;
    mp.centerOfMass /= (6 * polygon.area);

    return mp;
}