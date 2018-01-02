//
// Created by Kevin Yu on 2016-07-18.
//
#pragma once

template <typename T>
class ftVectorArray {
public:

    void init(uint32 capacity);
    void cleanup();

    T& operator[](const uint32 idx) const;

    void push(T obj);
    uint32 push();
    void reserve(uint32 size);
    void remove();
    void removeAll();

    uint32 getCapacity() const;
    uint32 getSize() const;

private:

    T* m_objects;
    uint32 m_nObject;
    uint32 m_capacity;

};

template <typename T>
void ftVectorArray<T>::init(uint32 capacity) {
    m_capacity = capacity;
    m_nObject = 0;
    m_objects = new T[m_capacity];
}

template <typename T>
void ftVectorArray<T>::cleanup() {
    m_capacity = 0;
    m_nObject = 0;
    delete[] m_objects;
}

template <typename T>
T& ftVectorArray<T>::operator[](const uint32 idx) const {
    ftAssert(idx < m_nObject,"");
    return m_objects[idx];
}

template <typename T>
void ftVectorArray<T>::push(T obj) {
    if (m_nObject == m_capacity) {
        T* old = m_objects;
        uint32 oldCapacity = m_capacity;
        m_capacity *= 2;
        m_objects = new T[m_capacity];
        memcpy(m_objects, old, oldCapacity * sizeof(T));
    }
    m_objects[m_nObject] = obj;
    ++m_nObject;
}

template <typename T>
uint32 ftVectorArray<T>::push() {
    if (m_nObject == m_capacity) {
        T* old = m_objects;
        uint32 oldCapacity = m_capacity;
        m_capacity *= 2;
        m_objects = new T[m_capacity];
        memcpy(m_objects, old, oldCapacity * sizeof(T));
    }
    ++m_nObject;
    return m_nObject-1;
}

template <typename T>
void ftVectorArray<T>::reserve(uint32 newCapacity) {
    if (newCapacity < m_capacity) return;
    else {
        T* old = m_objects;
        m_objects = new T[newCapacity];
        memcpy(m_objects, old, m_capacity * sizeof(T));
        m_capacity = newCapacity;
    }
}

template <typename T>
void ftVectorArray<T>::remove() {
    --m_nObject;
}

template <typename T>
void ftVectorArray<T>::removeAll() {
    m_nObject = 0;
}

template <typename T>
uint32 ftVectorArray<T>::getCapacity() const {
    return m_capacity;
}

template <typename T>
uint32 ftVectorArray<T>::getSize() const {
    return m_nObject;
}
