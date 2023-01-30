//
// Created by bismarck on 12/6/22.
//

#include <ros/ros.h>
#include "spectacular_ai/config.h"

config_pack getConfig() {
    config_pack config;
    ros::NodeHandle nh("~");

    nh.getParam("mxId", config.mxId);
    nh.getParam("oak/confidence", config.oak.confidence);
    nh.getParam("oak/LRchecktresh", config.oak.LRchecktresh);
    nh.getParam("oak/lrcheck", config.oak.lrcheck);
    nh.getParam("oak/extended", config.oak.extended);
    nh.getParam("oak/subpixel", config.oak.subpixel);
    nh.getParam("oak/colorResolution", config.oak.colorResolution);

    nh.getParam("vio/fastImu", config.oak.vio_config.fastImu);
    int accFrequencyHz, gyroFrequencyHz;
    nh.getParam("vio/accFrequencyHz", accFrequencyHz);
    nh.getParam("vio/gyroFrequencyHz", gyroFrequencyHz);
    config.oak.vio_config.accFrequencyHz = (unsigned)accFrequencyHz;
    config.oak.vio_config.gyroFrequencyHz = (unsigned)gyroFrequencyHz;
    nh.getParam("vio/inputResolution", config.oak.vio_config.inputResolution);

    nh.getParam("tf_prefix", config.ros.tfPrefix);
    nh.getParam("oak/angularVelCovariance", config.ros.angularVelCovariance);
    nh.getParam("oak/linearAccelCovariance", config.ros.linearAccelCovariance);

    return config;
}
