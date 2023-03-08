//
// Created by KevinTC.
//
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pcl_publisher;

void getcloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);
    cv::Mat hight_map(1200,1200,CV_8UC1,cv::Scalar(0));
    for(auto point : (pcl2cloud->points) ){
        if(point.x < 5.8){
            if(point.x > -5.8){
                if(point.y < 5.8){
                    if(point.y > -5.8){
                        if(point.z > -0.4){
                            if (point.z < 2){
                                int px = 100 * point.x + 600;
                                int py = 100 * point.y + 600;
                                unsigned char pz= 100 * point.z + 45;
                                hight_map.at<uchar>(px,py)=pz;
                            }
                        }
                    }
                }
            }
        }
    }
    cv::imshow("hight_map", hight_map);
    cv::waitKey(1);
    //ROS_INFO("%d",pcl2cloud->points.size());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_process");
    ros::NodeHandle pnh("~");
    auto subCloud = pnh.subscribe<sensor_msgs::PointCloud2>("/pointcloud2_in", 1, getcloud);
    pcl_publisher = pnh.advertise<sensor_msgs::PointCloud2>("/pointcloud2_out", 1);
    ros::spin();
    return 0;
    //auto sub1 = pnh.subscribe("/pointcloud2_in", 100, laserCallback);
    //auto sub2 = pnh.subscribe("/depth_image", 100, depth_img_callback);
    //pub = pnh.advertise<sensor_msgs::LaserScan>("/projected_scan", 1);
    //cv::destroyWindow("depth");
}
