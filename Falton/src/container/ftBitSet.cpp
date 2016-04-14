//
// Created by Kevin Yu on 3/19/16.
//

#include "falton/container/ftBitSet.h"
#include "string.h"

int ftBitSet::table_index(int index) {
    return index/8;
}

int ftBitSet::table_offset(int index) {
    return index%8;
}

ftBitSet::ftBitSet(int size) {

    int nChar = (size/8) + 1;

    bitTable = new char [nChar];
    capacity = nChar * 8;
    memset(bitTable,0,nChar);

}

ftBitSet::~ftBitSet() {
    delete bitTable;
}

bool ftBitSet::test(int index) {
    return bitTable[table_index(index)] & (1 << (table_offset(index)));
}

void ftBitSet::on(int index) {
    bitTable[table_index(index)] |= (1 << (table_offset(index)));
}

void ftBitSet::off(int index) {
    bitTable[table_index(index)] &= (~(1 << (table_offset(index))));
}

void ftBitSet::resize(int size) {
    int nChar = (size/8) + 1;

    int nBytesToCopy = nChar;
    if (nChar > capacity/8) nBytesToCopy = capacity/8;

    char *oldBitTable = bitTable;
    bitTable = new char [nChar];

    memcpy(bitTable, oldBitTable, nBytesToCopy);

    delete oldBitTable;

    capacity = nChar * 8;
}

int ftBitSet::getCapacity() {
    return capacity;
}