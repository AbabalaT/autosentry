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

struct PointInt {
    int p_x, p_y, p_z;
};

void getcloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    std::vector <std::vector<int>> point_list1(1200);
    std::vector <PointInt> point_list2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);
    cv::Mat hight_map(1201, 1201, CV_8UC1, cv::Scalar(0));
    cv::Mat gradient_map(1201, 1201, CV_8UC1, cv::Scalar(0));
    for (auto point: (pcl2cloud->points)) {
        if (point.x < 5.8) {
            if (point.x > -5.8) {
                if (point.y < 5.8) {
                    if (point.y > -5.8) {
                        if (point.z > -1.2) {
                            if (point.z < 1.2) {
                                if (point.x < 0.4 and point.x > -0.4) {
                                    if (point.y < 0.4 and point.y > -0.4) {
                                        continue;
                                    }
                                }
                                int px = 100 * point.x + 600;
                                int py = 100 * point.y + 600;
                                unsigned char pz = 100 * point.z + 130;
                                if (!(hight_map.at<uchar>(px, py))) {
                                    hight_map.at<uchar>(px, py) = pz;
                                    point_list1[px].push_back(py);
                                    point_list2.push_back({px, py, pz});
                                } else {
                                    if (pz < hight_map.at<uchar>(px, py)) {
                                        hight_map.at<uchar>(px, py) = pz;
                                        point_list1[px].push_back(py);
                                        point_list2.push_back({px, py, pz});
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    for (auto point: point_list2) {
        unsigned char max = 0;
        unsigned char min = 255;
        for (int surround_point_x = (((point.p_x - 10) > 0) ? (point.p_x - 10) : 0);
                surround_point_x < (((point.p_x + 10) < 1199) ? (point.p_x + 10) : 1199);
                surround_point_x = surround_point_x + 1) {
            for (auto surround_point_y : point_list1[surround_point_x]) {
                if((surround_point_y < point.p_y + 10) and (surround_point_y > point.p_y - 10)){
                    if(hight_map.at<uchar>(surround_point_x, surround_point_y) < min){
                        min = hight_map.at<uchar>(surround_point_x, surround_point_y);
                    }
                    if(hight_map.at<uchar>(surround_point_x, surround_point_y) > max){
                        max = hight_map.at<uchar>(surround_point_x, surround_point_y);
                    }
                }
                if(surround_point_y > point.p_y + 10){
                    break;
                }
            }
        }
        gradient_map.at<uchar>(point.p_x, point.p_y) = max - min;
    }
    cv::imshow("hight_map", gradient_map);
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
