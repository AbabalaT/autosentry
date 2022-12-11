//
// Created by bismarck on 12/8/22.
//

#include <iostream>
#include <utility>

#include <ros/ros.h>

#include "robot_driver//serialPort.h"

using std::cout, std::endl;

bool serialPort::init() {
    if (name.empty()) {
        serial.setPort("/dev/ttyUSB0");
        serial.setBaudrate(230400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial.setTimeout(to);
        serial.open();
        cout << "SYSTEM USB NOT DETECTED" << endl;
        if (!serial.isOpen()) {
            serial.setPort("/dev/ttyUSB1");
            serial.open();
        }
    } else {
        cout << "usb name is" << name << endl;
        serial.setPort(name);
        serial.setBaudrate(230400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial.setTimeout(to);
        serial.open();

    }

    //检测串口是否已经打开，并给出提示信息
    if (serial.isOpen()) {
        cout << "Serial Port initialized";
        return true;
    } else {
        cout << "Unable to open port ";
        return false;
    }
}

serialPort::serialPort(std::string _name) {
    name = std::move(_name);
    init();
    if (!listenerThread.joinable()) {
        listenerThread = std::thread(&serialPort::loop, this);
    }
}

serialPort::~serialPort() {
    if (serial.isOpen()) {
        serial.close();
    }
}

void serialPort::loop() {
    std::cout << "listener thread start" << std::endl;
    while (ros::ok()) {
        uint8_t data;
        try {
            serial.read(&data, 1);
            listener.append(*(char*)&data);
        } catch (serial::IOException &e) {
            init();
            std::cout << "serial read error" << std::endl;
            continue;
        }
    }
}
