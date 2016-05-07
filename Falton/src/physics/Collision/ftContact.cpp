//
// Created by Kevin Yu on 4/15/16.
//

#include <cstring>
#include "falton/physics/Collision/ftContact.h"

ftContactBuffer::ftContactBuffer() {
    contacts = new ftContactElem*[128];

    for (int i=0;i<128;++i) {
        contacts[i] = nullptr;
    }

    capacity = 128;
    nContact = 0;
}

ftContactBuffer::~ftContactBuffer() {
    for (uint32 i=0;i<capacity;i++) {
        ftContactElem *nextContact = contacts[i];
        while (nextContact) {
            ftContactElem *contact = nextContact;
            nextContact = nextContact->next;
            delete contact;
        }
    }

    delete[] contacts;
}

ftContact* ftContactBuffer::create(ftColHandle key1, ftColHandle key2) {
    ftContactElem *contactElem = new ftContactElem;

    nContact++;

    if (nContact > capacity) {
        rehash(nContact * 2);
    }

    if (key1 < key2) {
        contactElem->contact.handleA = key1;
        contactElem->contact.handleB = key2;
    } else {
        contactElem->contact.handleA = key2;
        contactElem->contact.handleB = key1;
    }

    uint32 hashValue = hash(contactElem->contact.handleA,
                            contactElem->contact.handleB, capacity);

    add(contactElem, hashValue);

    return &(contactElem->contact);
}

void ftContactBuffer::add(ftContactElem *elem, uint32 hashVal) {
    if (contacts[hashVal]!=nullptr) contacts[hashVal]->prev = elem;
    elem->next = contacts[hashVal];
    elem->prev = nullptr;
    contacts[hashVal] = elem;
    elem->hashValue = hashVal;
}

void ftContactBuffer::destroy(ftContact *contact) {
    ftContactElem* elem = (ftContactElem*) contact;
    ftContactElem* next = elem->next;
    ftContactElem* prev = elem->prev;

    if (prev!= nullptr) prev->next = next;
    else contacts[elem->hashValue] = next;

    if (next != nullptr) next->prev = prev;

    --nContact;

    delete elem;
}

uint32 ftContactBuffer::getSize() {

    return nContact;

}

ftContact* ftContactBuffer::find(ftColHandle key1, ftColHandle key2) {

    ftColHandle keyA;
    ftColHandle keyB;

    if (key1 < key2) {
        keyA = key1;
        keyB = key2;
    } else {
        keyA = key2;
        keyB = key1;
    }

    uint32 hashVal = hash(keyA, keyB, capacity);
    ftContactElem *nextContact = contacts[hashVal];
    while(nextContact) {
        if (nextContact->contact.handleA == keyA && nextContact->contact.handleB == keyB) return &(nextContact->contact);
        nextContact = nextContact->next;
    }

    return nullptr;

}

uint32 ftContactBuffer::hash(ftColHandle keyA, ftColHandle keyB, uint32 capacity) {
    uint64 x = (uint64) keyA ^ (uint64) keyB;
    return x % capacity;
}

uint64 ftContactBuffer::pairingFunction(ftColHandle keyA, ftColHandle keyB) {
    return (uint64) keyA ^ (uint64) keyB;
}

void ftContactBuffer::rehash(uint32 newCapacity) {

    ftContactElem **oldBuffer = contacts;
    contacts = new ftContactElem*[newCapacity];

    for (uint32 i = 0; i < newCapacity; ++i) {
        contacts[i] = nullptr;
    }

    for (uint32 i=0;i<capacity;i++) {
        ftContactElem *nextContact = oldBuffer[i];
        while(nextContact) {
            uint32 hashVal = hash(nextContact->contact.handleA, nextContact->contact.handleB, newCapacity);
            ftContactElem *contact = nextContact;
            nextContact = nextContact->next;
            add(contact, hashVal);
        }
    }

    delete[] oldBuffer;

    capacity = newCapacity;
}

ftContactBuffer::ftIter ftContactBuffer::iterate() {

    ftIter iter;

    iter.index = capacity-1;
    iter.nextContact = nullptr;

    for (uint32 i = 0; i < capacity; ++i) {
        ftContactElem *nextContact = contacts[i];
        if (nextContact!=nullptr) {
            iter.index = i;
            iter.nextContact = nextContact;
            break;
        }
    }

    return iter;

}

ftContact* ftContactBuffer::start(ftIter* iter) {
    ftContact* contact = (ftContact*) iter->nextContact;
    return contact;
}

ftContact* ftContactBuffer::next(ftIter* iter) {

    if (iter->nextContact->next != nullptr) {
        iter->nextContact = iter->nextContact->next;
    } else {
        int start = iter->index + 1;
        iter->index = capacity-1;
        iter->nextContact = nullptr;
        for (uint32 i = start; i < capacity; ++i) {
            ftContactElem *nextContact = contacts[i];
            if (nextContact != nullptr) {
                iter->index = i;
                iter->nextContact = nextContact;
                break;
            }
        }
    }

    ftContact* contact = (ftContact*) iter->nextContact;
    return contact;
}