//
// Created by Kevin Yu on 2016-07-02.
//

#ifndef FALTON_FTSTACK_H
#define FALTON_FTSTACK_H

#include <falton/setting/general.h>

template <typename Data>
class ftStack {
public:
    void init(uint32 size);
    void cleanup();
    void push(Data datum);
    void add();
    Data pop();
    Data& top();

    uint32 getSize();
    uint32 getCapacity();

    ~ftStack();

private:
    Data *data;
    uint32 m_capacity;
    uint32 m_nObject;
};

template <typename Data>
void ftStack<Data>::init(uint32 capacity) {
    m_capacity = capacity;
    data = new Data[m_capacity];
    m_nObject = 0;
}

template <typename Data>
void ftStack<Data>::cleanup() {
    m_capacity = 0;
    m_nObject = 0;
    delete[] data;
}

template <typename Data>
ftStack<Data>::~ftStack() {
    ftAssert(m_capacity == 0 && m_nObject == 0, "No message");
}

template <typename Data>
void ftStack<Data>::push(Data datum) {
    ftAssert(m_nObject + 1 <= m_capacity, "N Object : "<<m_nObject<<", Capacity : "<<m_capacity);
    data[m_nObject] = datum;
    ++m_nObject;
}

template <typename Data>
void ftStack<Data>::add() {
    ftAssert(m_nObject + 1 <= m_capacity, "N Object : "<<m_nObject<<", Capacity : "<<m_capacity);
    ++m_nObject;
}

template <typename Data>
Data ftStack<Data>::pop() {
    ftAssert(m_nObject > 0, "N Object : "<<m_nObject);
    --m_nObject;
    return data[m_nObject];
}

template <typename Data>
Data& ftStack<Data>::top() {
    ftAssert(m_nObject > 0, "N Object : "<<m_nObject);
    return data[m_nObject-1];
}

template <typename Data>
uint32 ftStack<Data>::getSize() {
    return m_nObject;
}

template <typename Data>
uint32 ftStack<Data>::getCapacity() {
    return m_capacity;
}

#endif //FALTON_FTSTACK_H
