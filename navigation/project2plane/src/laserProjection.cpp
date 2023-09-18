//
// Created by bismarck on 12/11/22.
//
#define USE_CUSTOM_LASER2SCAN

#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>



#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

ros::Publisher pub;
bool publish_tf = false;

bool callbacked_flag = false;
tf2_ros::Buffer tfBuffer;

void project2plane_callback(const ros::TimerEvent&){    //将3D位置投影到2D地图上用于导航
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped laser2base, base2map;
    try {
        base2map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Project2Scan Get TF ERROR!");
        return;
    }

    tf2::Quaternion b2m{base2map.transform.rotation.x, base2map.transform.rotation.y,
                        base2map.transform.rotation.z, base2map.transform.rotation.w};
    double roll = 0, pitch = 0, yaw = 0;
    tf2::Matrix3x3(b2m).getRPY(roll, pitch, yaw);

    if (publish_tf) {
        geometry_msgs::TransformStamped trans = base2map;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        trans.child_frame_id = "plane_base_link";
        trans.header.stamp = ros::Time::now();
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();
        trans.transform.translation.z = 0;
        br.sendTransform(trans);
    }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {   //根据当前姿态对线性激光结果进行转换
    sensor_msgs::LaserScan new_msg = *msg;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped laser2base, base2map;
    try {
        base2map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Project2Scan Get TF ERROR!");
        return;
    }

    tf2::Quaternion b2m{base2map.transform.rotation.x, base2map.transform.rotation.y,
                        base2map.transform.rotation.z, base2map.transform.rotation.w};
    double roll = 0, pitch = 0, yaw = 0;
    tf2::Matrix3x3(b2m).getRPY(roll, pitch, yaw);
    float theta = new_msg.angle_min;
    for (auto &it: new_msg.ranges) {
        it = it * cos(pitch*cos(theta));
        theta = theta + new_msg.angle_increment;
    }
    //ROS_INFO("Start Angle: %f, End Angle: %f", new_msg.angle_min, new_msg.angle_max);
    new_msg.header.frame_id = "plane_" + new_msg.header.frame_id;
    for(int i = 0; i<10; i=i+1){
        new_msg.ranges[i]=0.0;
    }
    for(int i = 631; i<640; i=i+1){
        new_msg.ranges[i]=0.0;
    }
    pub.publish(new_msg);

}

void depth_img_callback(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat cv_img0 =  cv_bridge::toCvCopy(msg, "16UC1")->image;
    cv::imshow("depth", cv_img0);
    cv::waitKey(1);
    /*
    sensor_msgs::LaserScan new_msg;
    。。。处理深度图到扫瞄。。。
    。。。投影转平面。。。
    */
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_projection");
    ros::NodeHandle pnh("~");
    pnh.param<bool>("publish_tf", publish_tf, false);
    tf2_ros::TransformListener tfListener(tfBuffer);
//#ifndef USE_CUSTOM_LASER2SCAN
    auto sub1 = pnh.subscribe("/scan", 100, laserCallback);
    ros::Timer timer1 = pnh.createTimer(ros::Duration(0.02), project2plane_callback);
//    auto odomSub = pnh.subscribe<nav_msgs::Odometry>("/Odometry", 10, lidar_to_odom);
//#else
    //auto sub2 = pnh.subscribe("/depth_image", 100, depth_img_callback);
//#endif
    pub = pnh.advertise<sensor_msgs::LaserScan>("/projected_scan", 1);
    ros::spin();
//    cv::destroyWindow("depth");
    return 0;
}
