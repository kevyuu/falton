//
// Created by Kevin Yu on 4/17/16.
//

#ifndef FALTON_FTISLANDCOMPUTER_H
#define FALTON_FTISLANDCOMPUTER_H

#include <falton/container/ftChunkArray.h>
#include <falton/dynamic/ftJoint.h>

struct ftBody;
struct ftContact;

class ftBodyBuffer;

struct ftIsland {
    ftChunkArray<ftBody*> bodies;
    ftChunkArray<ftContact*> contacts;
    ftChunkArray<ftJoint*> joints;
};



#endif //FALTON_FTISLANDCOMPUTER_H
