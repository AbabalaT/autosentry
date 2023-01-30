//
// Created by bismarck on 12/3/22.
//
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <depthai/depthai.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include <depthai_bridge/BridgePublisher.hpp>


int main(int argc, char** argv) {
    ros::init(argc, argv, "oak_imu");
    ros::NodeHandle pnh;

    dai::Pipeline pipeline;
    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    xoutImu->setStreamName("imu");

    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);

    imu->out.link(xoutImu->input);

    dai::Device device(pipeline);

    auto imuQueue = device.getOutputQueue("imu", 30, false);

    dai::rosBridge::ImuConverter imuConverter("oak");
    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
            imuQueue,
            pnh,
            std::string("imu"),
            std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            "",
            "imu");

    imuPublish.addPublisherCallback();

    ros::spin();

    return 0;
}
