//
// Created by Kevin Yu on 2016-05-20.
//

#ifndef FALTON_FTSHAPEBUFFER_H
#define FALTON_FTSHAPEBUFFER_H

#include <falton/shape/ftPolygon.h>
#include <falton/shape/ftCircle.h>

struct ftShape;

class ftShapeBuffer {
public:
    void init();
    void cleanup();

    ftShape* createShape(ftShape* shape);
    ftPolygon* createPolygon();
    ftCircle* createCircle();

    void destroyShape(ftShape* shape);
    void destroyPolygon(ftPolygon* polygon);
    void destoryCircle(ftCircle* circle);

    template <typename T>
    void iterateShape(T func);

    template <typename T>
    void iteratePolygon(T func);

    template <typename T>
    void iterateCircle(T func);

private:
    const int32 NULL_SHAPE = -1;

    struct ftPolygonElem {
        ftPolygon polygon;
        int32 prev;
        int32 next;
    };

    struct ftCircleElem {
        ftCircle circle;
        int32 prev;
        int32 next;
    };

    ftChunkArray<ftPolygonElem> m_polygons;
    ftChunkArray<ftCircleElem> m_circles;

    int32 m_circleHead;
    int32 m_circleFree;

    int32 m_polygonHead;
    int32 m_polygonFree;

};

template <typename T>
inline void ftShapeBuffer::iterateShape(T func) {
    iterateCircle(func);
    iteratePolygon(func);
}

template <typename T>
inline void ftShapeBuffer::iterateCircle(T func) {
    for (int32 i = m_circleHead; i != NULL_SHAPE; i = m_circles[i].next) {
        func(&m_circles[i].circle);
    }
}

template <typename T>
inline void ftShapeBuffer::iteratePolygon(T func) {
    for (int32 i = m_polygonHead; i != NULL_SHAPE; i = m_polygons[i].next) {
        func(&m_polygons[i].polygon);
    }
}

#endif //FALTON_FTSHAPEBUFFER_H
