//
// Created by bismarck on 12/3/22.
//
#ifndef SRC_ROS_BRIDGE_H
#define SRC_ROS_BRIDGE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>

#include "spectacular_ai/oakInterface.h"

struct rosBridgeConfig {
    std::string tfPrefix = "oak";
    dai::ros::ImuSyncMethod imuMode = dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL;
    double angularVelCovariance = 0;
    double linearAccelCovariance = 0;
};

class ros_bridge {
private:
    ros::NodeHandle pnh;
    dai::rosBridge::ImuConverter imuConverter;
    dai::rosBridge::ImageConverter rgbConverter;
    dai::rosBridge::ImageConverter depthConverter;
    std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> rgbPublisher;
    dai::CalibrationHandler calibrationHandler;

    ros::Publisher imuPublisher;
    image_transport::Publisher depthPublisher;
    ros::Publisher depthCamInfoPublisher;

    sensor_msgs::CameraInfo depthCamInfo;
    oakConfig oak_config;

public:
    std::function<void(std::shared_ptr<dai::IMUData>)> imuPublish;
    std::function<void(std::shared_ptr<dai::ImgFrame>)> depthPublish;

    explicit ros_bridge(const rosBridgeConfig& config, const oakConfig& _oak_config, dai::CalibrationHandler _calibrationHandler);
    void imuPublishHelper(std::shared_ptr<dai::IMUData> inData);
    void depthPublishHelper(std::shared_ptr<dai::ImgFrame> inData);
    void registerRgbQueue(std::shared_ptr<dai::DataOutputQueue> queue);
};


#endif //SRC_ROS_BRIDGE_H
