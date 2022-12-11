//
// Created by bismarck on 12/8/22.
//

#ifndef MSG_SERIALIZE_WRITER_H
#define MSG_SERIALIZE_WRITER_H

#include <string>
#include <cstring>

#include "check.h"
#include "header_process.h"

template<typename T>
std::string serilize(int id, T t) {
    std::string s;
    s.resize(headerSize + sizeof(T) + checkSize);
    parseToHeader((uint8_t*)s.data(), id, sizeof(T));
    memcpy((void*)(s.data()+headerSize), &t, sizeof(T));
    s[sizeof(T)+headerSize] = checkFunc((uint8_t*)s.data(), sizeof(T)+headerSize);
    return std::move(s);
}

#endif //MSG_SERIALIZE_WRITER_H
