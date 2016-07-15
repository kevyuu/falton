//
// Created by Kevin Yu on 4/15/16.
//

#include <cstring>
#include <falton/physics/Collision/ftContact.h>

void ftContactBuffer::init() {
    m_hashTable = new ftContactElem*[128];

    for (int i=0;i<128;++i) {
        m_hashTable[i] = nullptr;
    }

    m_capacity = 128;
    m_nContact = 0;
    m_freeElementHead = nullptr;

    m_elements.init(128);
}

void ftContactBuffer::cleanup() {
    delete[] m_hashTable;
    m_elements.cleanup();
}

ftContact* ftContactBuffer::create(ftColHandle keyA, ftColHandle keyB) {
    ftContactElem *contactElem = allocateElement();

    ++m_nContact;

    if (m_nContact > m_capacity) {
        rehash(m_nContact * 2);
    }

    contactElem->handleA = keyA;
    contactElem->handleB = keyB;
    uint32 hashValue = hash(contactElem->handleA,
                            contactElem->handleB, m_capacity);

    add(contactElem, hashValue);

    return &(contactElem->contact);
}

void ftContactBuffer::add(ftContactElem *elem, uint32 hashVal) {
    if (m_hashTable[hashVal]!=nullptr) m_hashTable[hashVal]->prev = elem;
    elem->next = m_hashTable[hashVal];
    elem->prev = nullptr;
    m_hashTable[hashVal] = elem;
    elem->hashValue = hashVal;
}

void ftContactBuffer::destroy(ftContact *contact) {
    ftContactElem* elem = (ftContactElem*) contact;
    ftContactElem* next = elem->next;
    ftContactElem* prev = elem->prev;

    if (prev!= nullptr) prev->next = next;
    else m_hashTable[elem->hashValue] = next;

    if (next != nullptr) next->prev = prev;

    --m_nContact;

    freeElement(elem);
}

uint32 ftContactBuffer::getSize() {

    return m_nContact;

}

ftContact* ftContactBuffer::find(ftColHandle keyA, ftColHandle keyB) {

    uint32 hashVal = hash(keyA, keyB, m_capacity);
    ftContactElem *nextContact = m_hashTable[hashVal];
    while(nextContact) {
        if (nextContact->handleA == keyA && nextContact->handleB == keyB) return &(nextContact->contact);
        nextContact = nextContact->next;
    }

    return nullptr;

}

uint32 ftContactBuffer::hash(ftColHandle key1, ftColHandle key2, uint32 capacity) {
    key1 = ((key1 >> 16) ^ key1) * 0x45d9f3b;
    key1 = ((key1 >> 16) ^ key1) * 0x45d9f3b;
    key1 = ((key1 >> 16) ^ key1);
    key2 = ((key2 >> 16) ^ key2) * 0x45d9f3b;
    key2 = ((key2 >> 16) ^ key2) * 0x45d9f3b;
    key2 = ((key2 >> 16) ^ key2);
    int32 key = key1 ^ key2;
    return key % capacity;
}

uint64 ftContactBuffer::pairingFunction(ftColHandle keyA, ftColHandle keyB) {
    return (uint64) keyA ^ (uint64) keyB;
}

void ftContactBuffer::rehash(uint32 newCapacity) {

    ftContactElem **oldBuffer = m_hashTable;
    m_hashTable = new ftContactElem*[newCapacity];

    for (uint32 i = 0; i < newCapacity; ++i) {
        m_hashTable[i] = nullptr;
    }

    for (uint32 i=0;i<m_capacity;i++) {
        ftContactElem *nextContact = oldBuffer[i];
        while(nextContact) {
            uint32 hashVal = hash(nextContact->handleA, nextContact->handleB, newCapacity);
            ftContactElem *contact = nextContact;
            nextContact = nextContact->next;
            add(contact, hashVal);
        }
    }

    delete[] oldBuffer;

    m_capacity = newCapacity;
}

ftContactBuffer::ftContactElem* ftContactBuffer::allocateElement() {
    if (m_freeElementHead == nullptr) {
        int32 index = m_elements.add();
        return &m_elements[index];
    } else {
        ftContactElem* newElem = m_freeElementHead;
        m_freeElementHead = newElem->next;
        return newElem;
    }
}

void ftContactBuffer::freeElement(ftContactElem *elem) {
    elem->hashValue = -1;
    elem->next = m_freeElementHead;
    m_freeElementHead = elem;
}


//contact buffer3
void ftContactBuffer3::init() {
    hashTable.init(512);
    m_elements.init(128);
    m_freeElementHead = nullptr;
}

void ftContactBuffer3::cleanup() {
    hashTable.cleanup();
    m_elements.cleanup();
}

ftContact* ftContactBuffer3::create(ftColHandle key1, ftColHandle key2) {
    ftContactElem* elem = allocateElement();
    elem->handleA = key1;
    elem->handleB = key2;
    hashTable.insert(key1, key2, elem);
    return &elem->contact;
}

ftContact* ftContactBuffer3::find(ftColHandle key1, ftColHandle key2) {
    ftContact* contact = (ftContact*) hashTable.find(key1, key2);
    return contact;
}

void ftContactBuffer3::destroy(ftContact *contact) {
    ftContactElem* elem = (ftContactElem*) contact;
    hashTable.remove(elem->handleA, elem->handleB);
    freeElement(elem);
}

ftContactBuffer3::ftContactElem* ftContactBuffer3::allocateElement() {
    if (m_freeElementHead == nullptr) {
        int32 index = m_elements.add();
        return &m_elements[index];
    } else {
        ftContactElem* newElem = m_freeElementHead;
        m_freeElementHead = newElem->next;
        return newElem;
    }
}

void ftContactBuffer3::freeElement(ftContactElem *elem) {
    elem->isAllocated = -1;
    elem->next = m_freeElementHead;
    m_freeElementHead = elem;
}

uint32 ftContactBuffer3::getSize() {
    return hashTable.getSize();
}