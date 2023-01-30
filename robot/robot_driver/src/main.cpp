//
// Created by bismarck on 12/11/22.
//
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "robot_driver/serialPort.h"

ros::Publisher odomPub;
serialPort serial_handle;

message_data odometry {
    int x, y, yaw;
    int vx, vy, wz;
};

message_data cmd {
    int16_t vx, vy, wz;
};

void odomCallback(odometry msg) {
    //std::cout<<"Receive a frame of odom information!"<< std::endl;
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = msg.x / 1000.;
    odom.pose.pose.position.y = msg.y / 1000.;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = msg.vx / 1000.;
    odom.twist.twist.linear.y = msg.vy / 1000.;
    odom.twist.twist.angular.z = msg.wz / 1000.;
    odom.twist.covariance = {
            1e-2, 0, 0, 0, 0, 0,
            0, 1e-2, 0, 0, 0, 0,
            0, 0, 1e-2, 0, 0, 0,
            0, 0, 0, 1e-2, 0, 0,
            0, 0, 0, 0, 1e-2, 0,
            0, 0, 0, 0, 0, 1e-2
    };
//    std::cout
//        <<odom.pose.pose.position.x<<" "
//        <<odom.pose.pose.position.y<<" "
//        <<odom.twist.twist.linear.x<<" "
//        <<odom.twist.twist.linear.y<<" "
//        <<odom.twist.twist.angular.z<<" "<<std::endl;
    odomPub.publish(odom);
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    std::cout<<"receive a control speed!"<<std::endl;
    cmd c {
        .vx = (int16_t)(msg->linear.x * 1000),
        .vy = (int16_t)(msg->linear.y * 1000),
        .wz = (int16_t)(msg->angular.z * 1000)
    };
    std::cout<<std::dec
        <<c.vx<<" "
        <<c.vy<<" "
        <<c.wz<<std::endl;
    serial_handle.sendMsg(0x82, c);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh("~");

    nh.param<std::string>("serial_name", serial_handle.name, "/dev/ttyUSB0");
    auto sub = nh.subscribe("/robot/cmd_vel", 1, cmdCallback);
    odomPub = nh.advertise<nav_msgs::Odometry>("/robot/odom", 5);
    serial_handle.init();
    serial_handle.registerCallback<odometry>(0x12, odomCallback);
    ros::spin();
}
