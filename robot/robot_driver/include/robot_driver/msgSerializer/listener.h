//
// Created by bismarck on 12/8/22.
//

#ifndef MSG_SERIALIZE_LISTENER_H
#define MSG_SERIALIZE_LISTENER_H

#include <string>
#include <cstring>

#include "callback_manager.h"
#include "header_process.h"
#include "check.h"

class Listener {
private:
    CallbackManager callbackManager;
    std::string txBuffer{100};

public:
    template<typename T>
    bool registerCallback(int id, std::function<void(T)> userCallback) {
        return callbackManager.registerCallback(id, [userCallback](const char* data) {
            T t;
            memcpy(&t, data, sizeof(T));
            userCallback(t);
        });
    }

    bool append(const char c) {
        txBuffer.push_back(c);
        int eraseSize = 0;
        for (int i = 0; i < (long)txBuffer.size()-headerSize-checkSize; i++) {
            int id, size;
            if (parseFromHeader((uint8_t*)txBuffer.data()+i, (int)txBuffer.size()-i, id, size)) {
                size -= headerSize + checkSize;
                if (txBuffer.size() - i >= size + headerSize + checkSize) {
                    if (checkFunc((uint8_t*)txBuffer.data()+i, size+headerSize) ==
                            ((uint8_t*)txBuffer.data())[size+headerSize+i]) {
                        callbackManager[id](txBuffer.data()+i+headerSize);
                        txBuffer.erase(0, i+size);
                        return true;
                    } else {
                        eraseSize = i+1;
                    }
                }
            }
        }
        txBuffer.erase(0, eraseSize);
        return false;
    }
};

#endif //MSG_SERIALIZE_LISTENER_H
