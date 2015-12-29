//
// Created by Kevin Yu on 12/26/15.
//

#ifndef FALTON_FTHANDLE_H
#define FALTON_FTHANDLE_H

struct ftShapeHandle {
    Box* id;
};

struct ftBodyHandle {
    int id;
};

struct ftColliderHandle {
    ftCollider* id;
};

#endif //FALTON_FTHANDLE_H
