//
// Created by Kevin Yu on 4/20/16.
//
#pragma once

#include <falton/container/ftChunkArray.h>
#include <functional>
#include <falton/dynamic/ftJoint.h>
#include <falton/dynamic/ftBody.h>

struct ftBody;
struct ftIsland;
struct ftContact;
struct ftContactEdge;

class ftBodyBuffer;

struct ftBodyBuffers {
    ftBodyBuffer* staticBuffer;
    ftBodyBuffer* kinematicBuffer;
    ftBodyBuffer* dynamicBuffer;
    ftBodyBuffer* sleepingBuffer;
};

struct ftIslandContact {
    ftContact* contact;
    ftContactEdge* contactEdgeA;
    ftContactEdge* contactEdgeB;
    bool dfsFlag;
};

struct ftIslandJoint {
    ftJoint* joint;
    ftJointEdge* jointEdgeA;
    ftJointEdge* jointEdgeB;
    bool dfsFlag;
};

class ftIslandSystem {

public:

    void init(ftBodyBuffers bodyBuffers);
    void shutdown();

    void addContact(ftContact* contact);
    void removeContact(ftContact* contact);

    void addJoint(ftJoint* joint);
    void removeJoint(ftJoint* joint);

    void buildAndProcessIsland(std::function<void(const ftIsland &)> func);

private:
    ftBodyBuffers m_buffers;
    ftChunkArray<ftIslandContact> m_islandContacts;
    ftChunkArray<ftIslandJoint> m_islandJoints;

    void resetContactFlag();
    void resetJointFlag();

    static ftContactEdge* createContactEdge(ftBody *body1, ftBody *body2, ftContact *contact);
    static void destroyContactEdge(ftContactEdge *contactEdge);

    static ftJointEdge* createJointEdge(ftBody* body1, ftBody* body2, ftJoint* joint);
    static void destroyJointEdge(ftJointEdge* jointEdge);

    static void resetIslandID(ftBodyBuffer* buffer);

};