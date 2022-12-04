//
// Created by bismarck on 12/3/22.
//
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include "spectacular_ai/oakInterface.h"
#include "spectacular_ai/ros_bridge.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "spectacular_ai");

    geometry_msgs::TransformStamped transform2map, transform2base;
    geometry_msgs::PoseStamped pose;
    tf2_ros::TransformBroadcaster tb;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    oakConfig oak_config{};
    oakInterface oak("", oak_config);
    ros_bridge bridge(rosBridgeConfig{}, oak_config, oak.readCalibration());
    bridge.registerRgbQueue(oak.getRgbQueue(30));
    oak.registerImuHook(bridge.imuPublish);
    oak.registerDepthHook(bridge.depthPublish);
    oak.start();

    ROS_INFO("VIO Start!!!");
    while(ros::ok()) {
        transform2map.header.frame_id = "map";
        transform2map.child_frame_id = "oak-d-base-frame";
        auto vioOut = oak.getOutput();
        transform2base = tfBuffer.lookupTransform("oak-d-base-frame", "oak_right_camera_optical_frame", ros::Time(0));
        transform2map.header.stamp = ros::Time::now();
        tf2::Quaternion frame2world(vioOut->pose.orientation.x, vioOut->pose.orientation.y, vioOut->pose.orientation.z, vioOut->pose.orientation.w);
        frame2world = tf2::Quaternion(-0.5, -0.5, 0.5, 0.5) * frame2world.inverse();
        transform2map.transform.rotation.x = frame2world.x();
        transform2map.transform.rotation.y = frame2world.y();
        transform2map.transform.rotation.z = -frame2world.z();
        transform2map.transform.rotation.w = frame2world.w();
        transform2map.transform.translation.x = -vioOut->pose.position.x;
        transform2map.transform.translation.y = -vioOut->pose.position.y;
        transform2map.transform.translation.z = vioOut->pose.position.z;
        tb.sendTransform(transform2map);
        std::cout << vioOut->asJson() << std::endl;
    }

    return 0;
}
