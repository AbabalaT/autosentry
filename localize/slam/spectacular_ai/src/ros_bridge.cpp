//
// Created by bismarck on 12/3/22.
//

#include <utility>
#include "spectacular_ai/ros_bridge.h"

ros_bridge::ros_bridge(const rosBridgeConfig &config, const oakConfig& _oak_config,
                       dai::CalibrationHandler _calibrationHandler) :
        pnh("~"),
        imuConverter(config.tfPrefix + "_imu_frame", config.imuMode, config.linearAccelCovariance,
                     config.angularVelCovariance),
        depthConverter(config.tfPrefix + "_right_camera_optical_frame", true),
        rgbConverter(config.tfPrefix + "_rgb_camera_optical_frame", false),
        imuPublisher(pnh.advertise<sensor_msgs::Imu>("imu", 200)),
        depthCamInfoPublisher(pnh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 20)) {
    imuPublish = std::bind(&ros_bridge::imuPublishHelper, this, std::placeholders::_1);
    depthPublish = std::bind(&ros_bridge::depthPublishHelper, this, std::placeholders::_1);
    oak_config = _oak_config;
    calibrationHandler = std::move(_calibrationHandler);
    image_transport::ImageTransport it(pnh);
    depthPublisher = it.advertise("depth", 20);
    int width=640, height=400;
    if (oak_config.vio_config.inputResolution == "720P") {
        width = 1280;
        height = 720;
    }
    depthCamInfo =
            depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, width, height);
}

void ros_bridge::imuPublishHelper(std::shared_ptr<dai::IMUData> inData) {
    std::deque<sensor_msgs::Imu> opMsgs;
    imuConverter.toRosMsg(inData, opMsgs);
    int mainSubCount = imuPublisher.getNumSubscribers();
    while (opMsgs.size()) {
        sensor_msgs::Imu currMsg = opMsgs.front();
        if (mainSubCount > 0) {
            imuPublisher.publish(currMsg);
        }
        opMsgs.pop_front();
    }
}

void ros_bridge::depthPublishHelper(std::shared_ptr<dai::ImgFrame> inData) {
    std::deque<sensor_msgs::Image> opMsgs;
    int infoSubCount = 0, mainSubCount = 0;
    infoSubCount = depthCamInfoPublisher.getNumSubscribers();
    mainSubCount = depthPublisher.getNumSubscribers();
    if(mainSubCount > 0 || infoSubCount > 0) {
        depthConverter.toRosMsg(inData, opMsgs);
        while (opMsgs.size()) {
            sensor_msgs::Image currMsg = opMsgs.front();
            if (mainSubCount > 0) {
                depthPublisher.publish(currMsg);
            }
            if (infoSubCount > 0) {
                auto localCameraInfo = depthCamInfo;
                localCameraInfo.header.stamp = currMsg.header.stamp;
                localCameraInfo.header.frame_id = currMsg.header.frame_id;
                depthCamInfoPublisher.publish(localCameraInfo);
            }
            opMsgs.pop_front();
        }
    }
}

void ros_bridge::registerRgbQueue(std::shared_ptr<dai::DataOutputQueue> queue) {
    int width = 1920;
    int height = 1080;
    if (oak_config.colorResolution == "4K") {
        width = 3840;
        height = 2160;
    } else if (oak_config.colorResolution == "12MP") {
        width = 4056;
        height = 3040;
    }
    auto rgbCameraInfo =
            rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, width, height);
    rgbPublisher = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
            queue,
            pnh,
            std::string("color/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1,
                      std::placeholders::_2),
            30,
            rgbCameraInfo,
            "color");
    rgbPublisher->addPublisherCallback();
}
