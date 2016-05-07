//
// Created by Kevin Yu on 3/19/16.
//

#ifndef FALTON_FTBITSET_H
#define FALTON_FTBITSET_H


class ftBitSet {
private:
    char* bitTable;
    int capacity;
    int nChar;

    int table_index(int index);
    int table_offset(int index);

public:
    ftBitSet(int size);
    ~ftBitSet();

    void clear();
    void resize(int size);
    bool test(int index);
    void on(int index);
    void off(int index);

    int getCapacity();

};


#endif //FALTON_FTBITSET_H
