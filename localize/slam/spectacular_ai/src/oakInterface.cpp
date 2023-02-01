//
// Created by bismarck on 12/3/22.
//

#include <ros/ros.h>
#include "spectacular_ai/oakInterface.h"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

oakInterface::oakInterface(const std::string& mxId, const oakConfig& oak_config) {
    // 初始化oak pipeline
    cameraRgb = pipeline.create<dai::node::ColorCamera>();

    xoutRgb = pipeline.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");

    dai::ColorCameraProperties::SensorResolution resolution =
            dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    if (oak_config.colorResolution == "4K") {
        resolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
    } else if (oak_config.colorResolution == "1080P") {
        resolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    } else if (oak_config.colorResolution == "12MP") {
        resolution = dai::ColorCameraProperties::SensorResolution::THE_12_MP;
    }
    cameraRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    cameraRgb->setResolution(resolution);
    cameraRgb->setInterleaved(false);
    cameraRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    cameraRgb->video.link(xoutRgb->input);

    // 初始化SpectacularAI
    vioPipeline = std::make_shared<spectacularAI::daiPlugin::Pipeline>(pipeline, oak_config.vio_config);

    // 寻找oak设备
    bool isDeviceFound = false;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();
    ROS_INFO("Listing available devices...");
    for(const auto& deviceInfo : availableDevices) {
        ROS_INFO("Device Mx ID: %s", deviceInfo.getMxId().c_str());
        if(deviceInfo.getMxId() == mxId) {
            if(deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
                isDeviceFound = true;
                device = std::make_shared<dai::Device>(pipeline, deviceInfo);
                break;
            } else if(deviceInfo.state == X_LINK_BOOTED) {
                throw std::runtime_error("DepthAI Device with MxId  \"" + mxId
                                         + "\" is already booted on different process.  \"");
            }
        } else if(mxId.empty()) {
            isDeviceFound = true;
            device = std::make_shared<dai::Device>(pipeline);
        }
    }
    if(!isDeviceFound) {
        throw std::runtime_error("DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
    }
    ROS_INFO("Device USB status: %s", usbStrings[static_cast<int32_t>(device->getUsbSpeed())].c_str());

}

void oakInterface::registerImuHook(const std::function<void(std::shared_ptr<dai::IMUData>)>& imuHook) {
    vioPipeline->hooks.imu = imuHook;
}

void oakInterface::registerDepthHook(const std::function<void(std::shared_ptr<dai::ImgFrame>)>& imgHook) {
    vioPipeline->hooks.depth = imgHook;
}

void oakInterface::set_IR_project(float mA){
    bool result = device -> setIrLaserDotProjectorBrightness(mA, -1);
    if(result){
        ROS_WARN("IR Laser Set Success!");
        ROS_WARN("IR Laser Set Success!");
        ROS_WARN("IR Laser Set Success!");
        ROS_WARN("IR Laser Set Success!");
    }else{
        ROS_WARN("IR Laser Set Fail!");
        ROS_WARN("IR Laser Set Fail!");
        ROS_WARN("IR Laser Set Fail!");
        ROS_WARN("IR Laser Set Fail!");
    }
}

std::shared_ptr<dai::DataOutputQueue> oakInterface::getRgbQueue(int maxSize, bool blocking) {
    return device->getOutputQueue("rgb", maxSize, blocking);
}

std::shared_ptr<const spectacularAI::VioOutput> oakInterface::getOutput() {
    return session->waitForOutput();
}

dai::CalibrationHandler oakInterface::readCalibration() {
    return device->readCalibration();
}

void oakInterface::start() {
    session = vioPipeline->startSession(*device);
}
