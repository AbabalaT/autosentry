//
// Created by bismarck on 12/3/22.
//
#include <functional>

#include <depthai/depthai.hpp>
#include <spectacularAI/depthai/plugin.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#ifndef SRC_OAKINTERFACE_H
#define SRC_OAKINTERFACE_H

struct oakConfig {
    int confidence=200;
    int LRchecktresh=5;
    bool lrcheck=true;
    bool extended=false;
    bool subpixel=true;
    // 1080P : 1920 × 1080
    // 4K    : 3840 × 2160
    // 12MP  : 4056 × 3040
    std::string colorResolution="1080P";
    spectacularAI::daiPlugin::Configuration vio_config{};
};

class oakInterface {
private:
    std::string mxId;
    std::string _namespace;
    dai::Pipeline pipeline;
    std::shared_ptr<dai::Device> device;
    std::shared_ptr<spectacularAI::daiPlugin::Pipeline> vioPipeline;
    std::unique_ptr<spectacularAI::daiPlugin::Session> session;
    std::shared_ptr<dai::node::ColorCamera> cameraRgb;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    std::shared_ptr<dai::node::XLinkOut> xoutRgb;
    std::shared_ptr<dai::node::XLinkOut> xoutDepth;

public:
    oakInterface() = default;
    explicit oakInterface(const std::string& mxId, const oakConfig& oak_config=oakConfig{});
    void registerImuHook(const std::function<void(std::shared_ptr<dai::IMUData>)>& imuHook);
    void registerDepthHook(const std::function<void(std::shared_ptr<dai::ImgFrame>)>& imgHook);
    std::shared_ptr<dai::DataOutputQueue> getRgbQueue(int maxSize=5, bool blocking=false);
    std::shared_ptr<const spectacularAI::VioOutput> getOutput();
    dai::CalibrationHandler readCalibration();
    void start();
};


#endif //SRC_OAKINTERFACE_H
