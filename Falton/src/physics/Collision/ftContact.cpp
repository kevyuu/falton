//
// Created by Kevin Yu on 4/15/16.
//

#include <cstring>
#include "falton/physics/Collision/ftContact.h"

#include <iostream>
using namespace std;

void ftContactBuffer::init() {
    contacts = new ftContactElem*[128];

    for (int i=0;i<128;++i) {
        contacts[i] = nullptr;
    }

    capacity = 128;
    nContact = 0;
}

void ftContactBuffer::cleanup() {
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

ftContact* ftContactBuffer::create(ftColHandle keyA, ftColHandle keyB) {
    ftContactElem *contactElem = new ftContactElem;

    ++nContact;

    if (nContact > capacity) {
        rehash(nContact * 2);
    }

    contactElem->handleA = keyA;
    contactElem->handleB = keyB;
    uint32 hashValue = hash(contactElem->handleA,
                            contactElem->handleB, capacity);

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

ftContact* ftContactBuffer::find(ftColHandle keyA, ftColHandle keyB) {

    uint32 hashVal = hash(keyA, keyB, capacity);
    ftContactElem *nextContact = contacts[hashVal];
    while(nextContact) {
        if (nextContact->handleA == keyA && nextContact->handleB == keyB) return &(nextContact->contact);
        nextContact = nextContact->next;
    }

    return nullptr;

}

uint32 ftContactBuffer::hash(ftColHandle keyA, ftColHandle keyB, uint32 capacity) {
    uint64 x = (uint64) keyA ^ (uint64) keyB;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x);
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
            uint32 hashVal = hash(nextContact->handleA, nextContact->handleB, newCapacity);
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
    iter.contact = nullptr;

    for (uint32 i = 0; i < capacity; ++i) {
        ftContactElem *nextContact = contacts[i];
        if (nextContact!=nullptr) {
            iter.index = i;
            iter.handleA = nextContact->handleA;
            iter.handleB = nextContact->handleB;
            iter.contact = (ftContact*)nextContact;
            break;
        }
    }

    return iter;

}

void ftContactBuffer::next(ftIter* iter) {

    ftContactElem* contactElem = (ftContactElem*) iter->contact;
    if (contactElem->next != nullptr) {
        iter->contact = (ftContact*) contactElem->next;
    } else {
        int start = iter->index + 1;
        iter->index = capacity-1;
        iter->contact = nullptr;
        for (uint32 i = start; i < capacity; ++i) {
            ftContactElem *nextContact = contacts[i];
            if (nextContact != nullptr) {
                iter->index = i;
                iter->handleA = nextContact->handleA;
                iter->handleB = nextContact->handleB;
                iter->contact = (ftContact*) nextContact;
                return;
            }
        }
    }
}

//Contact Buffer 2
void ftContactBuffer2::init() {
    contactIndex = new ftContactIndex[128];

    for (int i=0 ; i < 128;++i) {
        contactIndex[i].hashVal = -1;
        contactIndex[i].contactElem = nullptr;
    }

    capacity = 128;
    nContact = 0;
}

void ftContactBuffer2::cleanup() {

    for (int i = 0; i < capacity; ++i) {
        if (contactIndex[i].contactElem != nullptr) delete contactIndex[i].contactElem;
    }

    delete[] contactIndex;
}

ftContact* ftContactBuffer2::create(ftColHandle handleA, ftColHandle handleB) {
    ftContactElem *contactElem = new ftContactElem;

    ++nContact;

    if (nContact > 0.8f * capacity) {
        rehash(nContact * 2);
    }

    uint32 hashValue = hash(handleA, handleB, capacity);

    add(contactElem, hashValue, handleA, handleB);

    return &(contactElem->contact);
}

void ftContactBuffer2::add(ftContactElem *elem, uint32 hashValue, uint32 handleA, uint32 handleB) {
    uint32 index = hashValue;
    while (contactIndex[index].contactElem != nullptr) {
        ++index;
        index %= capacity;
    }
    contactIndex[index].hashVal = hashValue;
    contactIndex[index].handleA = handleA;
    contactIndex[index].handleB = handleB;
    contactIndex[index].contactElem = elem;

    elem->index = index;
}

void ftContactBuffer2::destroy(ftContact *contact) {
    ftContactElem* elem = (ftContactElem*) contact;
    uint32 index = elem->index;

    contactIndex[index].contactElem = nullptr;

    --nContact;

    delete elem;
}

uint32 ftContactBuffer2::getSize() {

    return nContact;

}

ftContact* ftContactBuffer2::find(ftColHandle keyA, ftColHandle keyB) {

    uint32 hashVal = hash(keyA, keyB, capacity);
    uint32 index = hashVal;
    while ((contactIndex[index].handleA !=keyA || contactIndex[index].handleB != keyB) &&
            contactIndex[index].hashVal != -1) {
        ++index;
        index %= capacity;
    }
    return (ftContact*) contactIndex[index].contactElem;

}

uint32 ftContactBuffer2::hash(ftColHandle keyA, ftColHandle keyB, uint32 capacity) {
    uint64 x = (uint64) keyA ^ (uint64) keyB;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x);
    return x % capacity;
}

uint64 ftContactBuffer2::pairingFunction(ftColHandle keyA, ftColHandle keyB) {
    return (uint64) keyA ^ (uint64) keyB;
}

void ftContactBuffer2::rehash(uint32 newCapacity) {

    ftContactIndex* oldIndex = contactIndex;
    contactIndex = new ftContactIndex[newCapacity];

    for (uint32 i = 0; i < newCapacity; ++i) {
        contactIndex[i].contactElem = nullptr;
        contactIndex[i].hashVal = -1;
    }

    for (uint32 i = 0; i < capacity; ++i) {
        if (oldIndex[i].contactElem != nullptr) {
            uint32 hashVal = hash(oldIndex[i].handleA, oldIndex[i].handleB, newCapacity);
            add(oldIndex[i].contactElem, hashVal, oldIndex[i].handleA, oldIndex[i].handleB);
        }
    }

    capacity = newCapacity;
    delete[] oldIndex;
}

ftContactBuffer2::ftIter ftContactBuffer2::iterate() {
    ftIter iter;

    iter.index = 0;
    while (iter.index < capacity && contactIndex[iter.index].contactElem == nullptr) {
        iter.index++;
    };

    if (iter.index == capacity) {
        iter.contact = nullptr;
    } else {
        iter.handleA = contactIndex[iter.index].handleA;
        iter.handleB = contactIndex[iter.index].handleB;
        iter.contact = (ftContact *) contactIndex[iter.index].contactElem;
    }

    return iter;

}

void ftContactBuffer2::next(ftIter* iter) {
    ++iter->index;
    while (iter->index < capacity && contactIndex[iter->index].contactElem == nullptr) {
        ++iter->index;
    };

    if (iter->index == capacity) {
        iter->contact = nullptr;
    } else {
        iter->handleA = contactIndex[iter->index].handleA;
        iter->handleB = contactIndex[iter->index].handleB;
        iter->contact = (ftContact *) contactIndex[iter->index].contactElem;
    }
}
