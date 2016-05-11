//
// Created by Kevin Yu on 4/20/16.
//

#ifndef FALTON_FTISLANDSYSTEM_H
#define FALTON_FTISLANDSYSTEM_H

#include <falton/container/ftChunkArray.h>
#include "ftBody.h"

struct ftBody;
struct ftIsland;
struct ftContact;
struct ftContactEdge;

class ftBodyBuffer;

struct ftBodyBuffers {
    ftBodyBuffer* staticBuffer;
    ftBodyBuffer* kinematicBuffer;
    ftBodyBuffer* dynamicBuffer;
};

struct ftIslandContact {
    ftContact* contact;
    ftContactEdge* contactEdgeA;
    ftContactEdge* contactEdgeB;
    bool dfsFlag;
};

class ftIslandSystem {

public:

    class ftIter {
    private:
        ftBodyBuffer::ftIter iter;
        ftIsland* prevIsland;
        friend class ftIslandSystem;
    };

    void init(ftBodyBuffers bodyBuffers);
    void shutdown();

    void addContact(ftContact* contact);
    void removeContact(ftContact* contact);

    ftIter iterate();
    ftIsland* start(ftIter *iter);
    ftIsland* next(ftIter *iter);

private:
    ftBodyBuffers m_buffers;
    ftChunkArray<ftIslandContact> m_islandContacts;

    void resetContactFlag();

    static ftContactEdge* createContactEdge(ftBody *body1, ftBody *body2, ftContact *contact);
    static void destroyContactEdge(ftContactEdge *contactEdge);
    static void resetIslandID(ftBodyBuffer* buffer);

};


#endif //FALTON_FTISLANDSYSTEM_H
