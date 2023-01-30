//
// Created by bismarck on 12/8/22.
//

#ifndef MSG_SERIALIZE_CALLBACK_MANAGER_H
#define MSG_SERIALIZE_CALLBACK_MANAGER_H

#include <iostream>
#include <map>
#include <functional>

#include <boost/serialization/singleton.hpp>
#include <utility>

class CallbackManager{
private:
    std::map<int, std::function<void(const char*)>> callbacks;
public:
    bool registerCallback(int id, std::function<void(const char*)> callback) {
        auto it = callbacks.find(id);
        if (it != callbacks.end()) {
            return false;
        }
        callbacks[id] = std::move(callback);
        return true;
    }

    std::function<void(const char*)> operator[](int id) {
        auto it = callbacks.find(id);
        if (it != callbacks.end()) {
            return it->second;
        } else {
            std::cout << "Command " << id << "not Registered" << std::endl;
            return [](const char*){};
        }
    }
};

#endif //MSG_SERIALIZE_CALLBACK_MANAGER_H
