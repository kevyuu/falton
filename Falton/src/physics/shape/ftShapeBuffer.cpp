//
// Created by Kevin Yu on 2016-05-20.
//

#include <falton/physics/shape/ftShapeBuffer.h>

void ftShapeBuffer::init() {

    m_circles.init(64);
    m_polygons.init(64);

    m_circleHead = NULL_SHAPE;
    m_polygonHead = NULL_SHAPE;
    m_circleFree = NULL_SHAPE;
    m_polygonFree = NULL_SHAPE;
}

void ftShapeBuffer::cleanup() {
    m_circles.cleanup();
    m_polygons.cleanup();
}

ftCircle* ftShapeBuffer::createCircle() {
    int32 index;
    if (m_circleFree == NULL_SHAPE) {
        index = m_circles.push();
    } else {
        index = m_circleFree;
        m_circleFree = m_circles[m_circleFree].next;
    }

    if (m_circleHead == NULL_SHAPE) {
        m_circleHead = index;
        m_circles[index].prev = NULL_SHAPE;
        m_circles[index].next = NULL_SHAPE;
    } else {
        m_circles[m_circleHead].prev = index;
        m_circles[index].prev = NULL_SHAPE;
        m_circles[index].next = m_circleHead;
        m_circleHead = index;
    }

    return &m_circles[index].circle;
}

ftPolygon* ftShapeBuffer::createPolygon() {
    int32 index;
    if (m_polygonFree == NULL_SHAPE) {
        index = m_polygons.push();
    } else {
        index = m_polygonFree;
        m_polygonFree = m_polygons[m_polygonFree].next;
    }

    if (m_polygonHead == NULL_SHAPE) {
        m_polygonHead = index;
        m_polygons[index].prev = NULL_SHAPE;
        m_polygons[index].next = NULL_SHAPE;
    } else {
        m_polygons[m_polygonHead].prev = index;
        m_polygons[index].prev = NULL_SHAPE;
        m_polygons[index].next = m_polygonHead;
        m_polygonHead = index;
    }

    return &m_polygons[index].polygon;
}

void ftShapeBuffer::destoryCircle(ftCircle *circle) {
    ftCircleElem* elem = (ftCircleElem*) circle;
    int32 next  = elem->next;
    int32 prev = elem->next;

    if (elem->next != NULL_SHAPE) {
        m_circles[next].prev = prev;
    }

    if (elem->prev == NULL_SHAPE) {
        m_circleHead = next;
    } else {
        m_circles[prev].next = next;
    }

    elem->next = m_circleFree;
    m_circleFree = elem->next;
}

void ftShapeBuffer::destroyPolygon(ftPolygon *polygon) {
    ftPolygonElem* elem = (ftPolygonElem*) polygon;
    int32 next = elem->next;
    int32 prev = elem->prev;

    if (elem->next != NULL_SHAPE) {
        m_polygons[next].prev = prev;
    }

    if (elem->prev == NULL_SHAPE) {
        m_polygonHead = next;
    } else {
        m_polygons[prev].next = next;
    }

    elem->next = m_polygonFree;
    m_polygonFree = elem->next;
}
