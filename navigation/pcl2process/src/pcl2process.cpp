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
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

ros::Publisher pcl_publisher;

inline float float_abs(float x){
    if(x > 0){
        return x;
    }else{
        return -x;
    }
}

struct PointInt {
    int p_x, p_y, p_z;
};

void getcloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    std::vector <std::vector<int>> point_list1(1200);
    std::vector <PointInt> point_list2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud <pcl::PointXYZ>);
    sensor_msgs::PointCloud2 ROSPCL_output;
    pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud <pcl::PointXYZ>);
    unsigned int point_num = 0;
    cv::Mat hight_map(1201, 1201, CV_8UC1, cv::Scalar(0));
    cv::Mat gradient_map(1201, 1201, CV_8UC1, cv::Scalar(0));
    for (auto point: (pcl2cloud->points)) {
        point.z = point.z + 0.52;
        if (point.x < 5.8) {
            if (point.x > -5.8) {
                if (point.y < 5.8) {
                    if (point.y > -5.8) {
                        if (point.z > -1.2) {
                            if (point.z < 1.2) {
                                if(point.x * point.x + point.y * point.y < 0.09){
                                    continue;
                                }
                                /*
                                float point_distance = sqrtf(point.x * point.x + point.y * point.y);
                                if(point_distance > 0.4f){
                                    if(float_abs(point.z) > 0.7f * (point_distance - 0.3f)){
                                        pcl::PointXYZ point4push;
                                        /*
                                        point4push.x = point.x;
                                        point4push.y = point.y;
                                        point4push.z = 0.0f;

                                        pcl2cloud_out->points.push_back(point);
                                        point_num = point_num + 1;
                                        continue;
                                    }
                                }
                                */
                                int px = 100 * point.x + 600;
                                int py = 100 * point.y + 600;
                                unsigned char pz = 100 * point.z + 130;
                                if (!(hight_map.at<uchar>(px, py))) {
                                    hight_map.at<uchar>(px, py) = pz;
                                    point_list1[px].push_back(py);
                                    point_list2.push_back({px, py, pz});
                                } else {
                                    if (pz > hight_map.at<uchar>(px, py)) {
                                        hight_map.at<uchar>(px, py) = pz;
                                        point_list1[px].push_back(py);
                                        point_list2.push_back({px, py, pz});
                                    }
                                }
                            }
                            //continue;
                        }
                    }
                }
            }
        }
        //pcl2cloud_out->points.push_back(point);
    }

    for (auto point: point_list2) {
        unsigned char max = 0;
        unsigned char min = 255;
        if(point.p_x < 635){
            if(point.p_x > 565){
                if(point.p_y < 635){
                    if(point.p_y > 565){
                        continue;
                    }
                }
            }
        }
        int surround_point_x = 0;
        for (surround_point_x = ((point.p_x - 10) > 0 ? (point.p_x - 10) : 0);
                surround_point_x < ((point.p_x + 10) < 1199 ? (point.p_x + 10) : 1199);
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
        unsigned char point_gradient = max - min;
        gradient_map.at<uchar>(point.p_x, point.p_y) = point_gradient;
        if(point_gradient > 14){
            pcl::PointXYZ point4push;
            point4push.x = (float)(point.p_x - 600)/100;
            point4push.y = (float)(point.p_y - 600)/100;
            point4push.z = 0.0f;
            pcl2cloud_out->points.push_back(point4push);
            point_num = point_num + 1;
        }else{
            unsigned char max_5cm = 0;
            unsigned char min_5cm = 255;
            int surround_point_x = 0;
            for (surround_point_x = ((point.p_x - 5) > 0 ? (point.p_x - 5) : 0);
                 surround_point_x < ((point.p_x + 5) < 1199 ? (point.p_x + 5) : 1199);
                 surround_point_x = surround_point_x + 1) {
                for (auto surround_point_y : point_list1[surround_point_x]) {
                    if((surround_point_y < point.p_y + 5) and (surround_point_y > point.p_y - 5)){
                        if(hight_map.at<uchar>(surround_point_x, surround_point_y) < min){
                            min_5cm = hight_map.at<uchar>(surround_point_x, surround_point_y);
                        }
                        if(hight_map.at<uchar>(surround_point_x, surround_point_y) > max){
                            max_5cm = hight_map.at<uchar>(surround_point_x, surround_point_y);
                        }
                    }
                    if(surround_point_y > point.p_y + 5){
                        break;
                    }
                }
            }
            unsigned char point_gradient_5cm = max_5cm - min_5cm;
            //gradient_map.at<uchar>(point.p_x, point.p_y) = point_gradient;
            if(point_gradient > 10){
                pcl::PointXYZ point4push;
                point4push.x = (float)(point.p_x - 600)/100;
                point4push.y = (float)(point.p_y - 600)/100;
                point4push.z = 0.0f;
                pcl2cloud_out->points.push_back(point4push);
                point_num = point_num + 1;
            }
        }
    }
    pcl2cloud_out->width = point_num;
    pcl2cloud_out->height = 1;
    pcl2cloud_out->points.resize(pcl2cloud_out->width * pcl2cloud_out->height);
    //cv::threshold(gradient_map, gradient_map, 10, 255, cv::THRESH_BINARY);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(pcl2cloud_out);
    filter.setLeafSize(0.03f, 0.03f, 0.03f);
    filter.filter(*pcl2cloud_out);

    pcl::toROSMsg(*pcl2cloud_out, ROSPCL_output);
    //cv::imshow("gradient_map", gradient_map);
    //cv::waitKey(1);
    ROSPCL_output.header.frame_id = "plane_base_link";
    pcl_publisher.publish(ROSPCL_output);
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
