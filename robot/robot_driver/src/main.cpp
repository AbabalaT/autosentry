//
// Created by bismarck on 12/11/22.
//
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "robot_driver/serialPort.h"

ros::Publisher odomPub;
serialPort serial_handle;

message_data odometry {
    float x, y, yaw;
    float vx, vy, wz;
};

message_data cmd {
    int16_t vx, vy, wz;
};

void odomCallback(odometry msg) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = msg.x;
    odom.pose.pose.position.y = msg.y;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = msg.vx;
    odom.twist.twist.linear.y = msg.vy;
    odom.twist.twist.angular.z = msg.wz;
    odom.twist.covariance = {
            1e-2, 0, 0, 0, 0, 0,
            0, 1e-2, 0, 0, 0, 0,
            0, 0, 1e-2, 0, 0, 0,
            0, 0, 0, 1e-2, 0, 0,
            0, 0, 0, 0, 1e-2, 0,
            0, 0, 0, 0, 0, 1e-2
    };
    odomPub.publish(odom);
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    cmd c {
        .vx = (int16_t)(msg->linear.x * 100),
        .vy = (int16_t)(msg->linear.y * 100),
        .wz = (int16_t)(msg->angular.z * 100)
    };
    serial_handle.sendMsg(0x82, c);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh("~");

    nh.param<std::string>("serial_name", serial_handle.name, "/dev/ttyUSB0");
    nh.subscribe("/robot/cmd_vel", 1, cmdCallback);
    odomPub = nh.advertise<nav_msgs::Odometry>("/robot/odom", 5);

    serial_handle.init();
    serial_handle.registerCallback<odometry>(0x12, odomCallback);

    ros::spin();
}
