//
// Created by bismarck on 12/8/22.
//

#ifndef RM2022ENGINEER_SERIALPORT_H
#define RM2022ENGINEER_SERIALPORT_H

#include <thread>
#include <serial/serial.h>

#include "msg_serialize.h"

class serialPort {
private:
    serial::Serial serial;
    Listener listener;
    std::thread listenerThread;

public:
    std::string name;
    serialPort() = default;
    explicit serialPort(std::string _name);
    ~serialPort();
    bool init();
    void loop();

    template<class T>
    void sendMsg(int id, T msg) {
        std::string s = serilize(id, msg);
        try {
            serial.write(s);
        } catch (serial::IOException &e) {
            init();
            std::cout << "serial write error" << std::endl;
        }
    }

    template<typename T>
    bool registerCallback(int id, std::function<void(T)> userCallback) {
        return listener.registerCallback(id, userCallback);
    }
};


#endif //RM2022ENGINEER_SERIALPORT_H
