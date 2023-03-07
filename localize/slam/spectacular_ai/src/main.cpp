//
// Created by bismarck on 12/3/22.
//
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include "spectacular_ai/oakInterface.h"
#include "spectacular_ai/ros_bridge.h"
#include "spectacular_ai/config.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "spectacular_ai");
    ros::NodeHandle nh("~");

    auto config = getConfig();
    geometry_msgs::TransformStamped transform2map, transform2base;
    nav_msgs::Odometry odom;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    oakInterface oak(config.mxId, config.oak);
    ros_bridge bridge(config.ros, config.oak, oak.readCalibration());
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 5);
    bridge.registerRgbQueue(oak.getRgbQueue(30));
    oak.registerImuHook(bridge.imuPublish);
    oak.registerDepthHook(bridge.depthPublish);
    //oak.set_IR_project(1000.0);
    oak.start();

    ROS_INFO("VIO Start!!!");
    while(ros::ok()) {
        auto vioOut = oak.getOutput();
        //ROS_INFO("VIO Running!!!");
        tf2::Quaternion frame2world(vioOut->pose.orientation.x, vioOut->pose.orientation.y, vioOut->pose.orientation.z, vioOut->pose.orientation.w);
        geometry_msgs::TransformStamped frame2base = tfBuffer.lookupTransform("base_link", "oak_front_right_camera_frame", ros::Time(0));
        frame2world = tf2::Quaternion(frame2base.transform.rotation.x, frame2base.transform.rotation.y,
                                      frame2base.transform.rotation.z, frame2base.transform.rotation.w).inverse()
                * tf2::Quaternion(-0.5, -0.5, 0.5, 0.5) * frame2world.inverse();

        std::string frame_id = "odom";
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = frame_id;
        odom.child_frame_id = frame_id;
        odom.pose.pose.position.x = -vioOut->pose.position.x ;
        odom.pose.pose.position.y = -vioOut->pose.position.y;
        odom.pose.pose.position.z = vioOut->pose.position.z;
        odom.pose.pose.orientation.x = frame2world.x();
        odom.pose.pose.orientation.y = frame2world.y();
        odom.pose.pose.orientation.z = -frame2world.z();
        odom.pose.pose.orientation.w = frame2world.w();
        odom.pose.covariance = {
                vioOut->positionCovariance[0][0], vioOut->positionCovariance[0][1], vioOut->positionCovariance[0][2], 0, 0, 0,
                vioOut->positionCovariance[1][0], vioOut->positionCovariance[1][1], vioOut->positionCovariance[1][2], 0, 0, 0,
                vioOut->positionCovariance[2][0], vioOut->positionCovariance[2][1], vioOut->positionCovariance[2][2], 0, 0, 0,
                0, 0, 0, 1e-2, 0, 0,
                0, 0, 0, 0, 1e-2, 0,
                0, 0, 0, 0, 0, 1e-2
        };
        odom.twist.twist.linear.x = vioOut->velocity.x;
        odom.twist.twist.linear.y = vioOut->velocity.y;
        odom.twist.twist.linear.z = -vioOut->velocity.z;
        odom.twist.twist.angular.x = vioOut->angularVelocity.x;
        odom.twist.twist.angular.y = vioOut->angularVelocity.y;
        odom.twist.twist.angular.z = vioOut->angularVelocity.z;
        odom.twist.covariance = {
                vioOut->velocityCovariance[0][0], vioOut->velocityCovariance[0][1], vioOut->velocityCovariance[0][2], 0, 0, 0,
                vioOut->velocityCovariance[1][0], vioOut->velocityCovariance[1][1], vioOut->velocityCovariance[1][2], 0, 0, 0,
                vioOut->velocityCovariance[2][0], vioOut->velocityCovariance[2][1], vioOut->velocityCovariance[2][2], 0, 0, 0,
                0, 0, 0, config.ros.angularVelCovariance, 0, 0,
                0, 0, 0, 0, config.ros.angularVelCovariance, 0,
                0, 0, 0, 0, 0, config.ros.angularVelCovariance
        };
        //std::cout<<"Velocity:"<<odom.twist.twist.linear.x<<" "<<odom.twist.twist.linear.y<<" "<<odom.twist.twist.linear.z<<std::endl;
        odom_pub.publish(odom);
        //std::cout << "Track Status: " << int(vioOut->status) << std::endl;
    }
    oak.set_IR_project(-1.0);
    return 0;
}
