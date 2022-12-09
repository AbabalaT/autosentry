//
// Created by bismarck on 12/6/22.
//

#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include "spectacular_ai/oakInterface.h"
#include "spectacular_ai/ros_bridge.h"

struct config_pack {
    std::string mxId;
    oakConfig oak;
    rosBridgeConfig ros;
};

config_pack getConfig();

#endif //SRC_CONFIG_H
