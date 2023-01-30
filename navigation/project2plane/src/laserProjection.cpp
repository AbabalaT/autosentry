//
// Created by bismarck on 12/11/22.
//

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher pub;
bool publish_tf = false;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static tf2_ros::TransformBroadcaster br;
    sensor_msgs::LaserScan new_msg = *msg;

    geometry_msgs::TransformStamped laser2base, base2map;
    try {
        base2map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    tf2::Quaternion b2m{base2map.transform.rotation.x, base2map.transform.rotation.y,
                        base2map.transform.rotation.z, base2map.transform.rotation.w};
    double roll, pitch, yaw;
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

    for (auto& it: new_msg.ranges) {
        it *= cos(pitch);
    }

    new_msg.header.frame_id = "plane_" + new_msg.header.frame_id;
    pub.publish(new_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_projection");
    ros::NodeHandle pnh("~");
    pnh.param<bool>("publish_tf", publish_tf, false);

    ros::Subscriber sub = pnh.subscribe("/scan", 1, laserCallback);
    pub = pnh.advertise<sensor_msgs::LaserScan>("/projected_scan", 1);

    ros::spin();
    return 0;
}
