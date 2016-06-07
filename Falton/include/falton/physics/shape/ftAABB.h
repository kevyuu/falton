//
// Created by Kevin Yu on 2/22/16.
//

#ifndef FALTON_FTAABB_H
#define FALTON_FTAABB_H

#include <falton/math/math.h>

struct ftAABB {
    ftVector2 min;
    ftVector2 max;

    bool overlap(const ftAABB& b) {
        if (this->max.x < b.min.x || b.max.x < this->min.x) return false;
        if (this->max.y< b.min.y || b.max.y < this->min.y) return false;
        return true;
    }

    void extend(real extension) {
        min.x -= extension;
        min.y -= extension;
        max.x += extension;
        max.y += extension;
    }

    real getPerimeter() {
        return 2 * ((max.x - min.x) + (max.y - min.y));
    }

    bool isContain(const ftAABB& b) {
        return (min.x < b.min.x && min.y < b.min.y && max.x > b.max.x && max.y > b.max.y);
    }

    uint32 getArea() {
        return (max.x - min.x) * (max.y - min.y);
    }

    static ftAABB combine(const ftAABB&a, const ftAABB&b) {
        ftAABB aabb;
        aabb.min.x = ftMin(a.min.x, b.min.x);
        aabb.min.y = ftMin(a.min.y, b.min.y);
        aabb.max.x = ftMax(a.max.x, b.max.x);
        aabb.max.y = ftMax(a.max.y, b.max.y);
        return aabb;
    }
};




#endif //FALTON_FTAABB_H
