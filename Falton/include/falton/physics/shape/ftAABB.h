//
// Created by Kevin Yu on 2/22/16.
//

#ifndef FALTON_FTAABB_H
#define FALTON_FTAABB_H


struct ftAABB {
    ftVector2 min;
    ftVector2 max;

    bool overlap(const ftAABB& b) {
        if (this->max.x < b.min.x || b.max.x < this->min.x) return false;
        if (this->max.y< b.min.y || b.max.y < this->min.y) return false;
        return true;
    }
};




#endif //FALTON_FTAABB_H
