//
// Created by bismarck on 12/8/22.
//

#ifndef MSG_SERIALIZE_HEADER_PROCESS_H
#define MSG_SERIALIZE_HEADER_PROCESS_H

#include "check.h"

const int headerSize = 3;

static bool parseFromHeader(const uint8_t* data, int length, int& id, int& size) {
    if (length < headerSize) {
        return false;
    }
    if (data[0] != 0xAA) {
        return false;
    }
    size = (int)data[1];
    id = (int)data[2];
    return true;
}

static void parseToHeader(uint8_t* data, int id, int size) {
    data[0] = 0xAA;
    data[1] = (uint8_t)size + headerSize + checkSize;
    data[2] = (uint8_t)id;
}

#endif //MSG_SERIALIZE_HEADER_PROCESS_H
