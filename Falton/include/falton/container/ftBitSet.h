//
// Created by Kevin Yu on 3/19/16.
//
#pragma once

#include "falton/math.h"

class ftBitSet {
private:
    char* bitTable = nullptr;
    uint32 capacity = 0;
    uint32 nChar = 0;

    uint32 table_index(int index);
    uint32 table_offset(int index);

public:

    void init(uint32 size);
    void cleanup();

    void clear();
    void resize(int size);
    bool test(int index);
    void on(int index);
    void off(int index);

    uint32 getCapacity();

};
