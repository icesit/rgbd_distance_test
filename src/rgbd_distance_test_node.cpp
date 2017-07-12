/*
program for distance test
by XueWuyang

 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "unistd.h"
#include "queue"

#include <sstream>
#include <iomanip>
#include "unistd.h"

//ROS头文件
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Vector3.h"

#include <time.h>
#include <math.h>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include "assistMath.h"

// 使用Eigen的Geometry模块处理3d运动
//#include <Eigen/Core>
//#include <Eigen/Geometry>

//#include "AstarPathPlanning2D.cpp"
//#include "Remotecontroller.h"
//#include "SJTUDrone.h"
//#include "uav_math.h"
//#include "postrajectory.h"

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointCloud<pcl::PointXYZ> pclPointCloudXYZ;
typedef boost::shared_ptr<pclPointCloudXYZ> pclPointCloudXYZPtr;

float translation[3]={0.0,0.0,0.0};
float qRotation[4]={0.0,0.0,0.0,0.0};
float eulerRotation[9]={0.0,0.0,0.0,
                        0.0,0.0,0.0,
                        0.0,0.0,0.0};
cv::Mat img_depth;
//camera info
float depth_info[3][4] = {575.8157348632812, 0.0, 314.5, 0.0,
                          0.0, 575.8157348632812, 235.5, 0.0,
                          0.0, 0.0, 1.0, 0.0};
//Subscriber and Publisher
ros::Subscriber depth_sub;
ros::Subscriber uav_sub;
ros::Subscriber stick_sub;
ros::Publisher pcl_pub;

//obstacle position
float stick_obs[3]={0.0,0.0,0.0};

//write into txt
std::ofstream totxt;

pcl::visualization::CloudViewer pclviewer("Cloud Viewer");

void viconCallback(const geometry_msgs::TransformStampedPtr& msg)
{
    qRotation[0] = msg->transform.rotation.w;
    qRotation[1] = msg->transform.rotation.x;
    qRotation[2] = msg->transform.rotation.y;
    qRotation[3] = msg->transform.rotation.z;
    translation[0] = msg->transform.translation.x;
    translation[1] = msg->transform.translation.y;
    translation[2] = msg->transform.translation.z;
    qToRotation(qRotation, eulerRotation);
}

void stickCallback(const geometry_msgs::TransformStampedPtr& msg)
{
    stick_obs[0] = msg->transform.translation.x;
    stick_obs[1] = msg->transform.translation.y;
    stick_obs[2] = msg->transform.translation.z;
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    img_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    //printf("image row is %d, col is %d\n", img_depth.rows, img_depth.cols);
    cv::imshow("image depth", img_depth);
    cv::waitKey(1);
    float *depthData, mindepth = 1000, mindist = 100, obs[2] ,tmpdist = 0,
          rawerror = 0.05;
    double sum[2]={0, 0};
    int cnt = 0;
    //pointcloud to world frame from camera frame
    pclPointCloudXYZ::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point1, pointW;
    float R[9];
    for(int i=0; i<img_depth.rows; ++i)
    {
        depthData=img_depth.ptr<float>(i);
        for(int j=0; j<img_depth.cols; ++j)
        {
            //abandom too far or too near
            if(depthData[j] > 5 || depthData[j] < 0.2 || isnan(depthData[j])) continue;
            point1.x = depthData[j];
            point1.y = -(j - depth_info[0][2]) * depthData[j] / depth_info[0][0];
            point1.z = -(i - depth_info[1][2]) * depthData[j] / depth_info[1][1];
//if(i==img_depth.rows/4 && j==img_depth.cols/4) printf("point cloud body x %f y %f z %f\n", point1.x, point1.y, point1.z);            
            //transform to world frame
            transform_NWUworld_from_body(point1.x, point1.y, point1.z,
                                         pointW.x, pointW.y, pointW.z,
                                         eulerRotation, translation);
//if(i==img_depth.rows/2 && j==img_depth.cols/2) printf("point cloud world x %f y %f z %f\n", pointW.x, pointW.y, pointW.z);
            if(fabs(pointW.z-translation[2])<0.1)
            {               
                tmpdist = sqrt(pow((pointW.x-translation[0]),2)
                               +pow((pointW.y-translation[1]),2));
                if(tmpdist>0.9) tmpdist = 0.05 * pow(tmpdist,2) + tmpdist;
                
                ++cnt;
                sum[0] += depthData[j];
                pCloud->points.push_back(pointW);
                //minimum distance
                if(tmpdist < mindist)
                {   
                    obs[0] = pointW.x;
                    obs[1] = pointW.y;
                    mindist = tmpdist;
                }
                if(depthData[j] < mindepth)
                {
                    mindepth = depthData[j];
                }
            }
        }
    }

    pCloud->height = 1;
    pCloud->width = pCloud->points.size();
    pCloud->is_dense = false;
    pclviewer.showCloud(pCloud);
    pCloud->points.clear();
    //printf("center distance is %f\n",img_depth.ptr<float>(img_depth.rows/2)[img_depth.cols/2]);
    //printf("the max depth distance is %f, min depth distance is %f\n", maxdepth, mindepth);
    float aver[2] = {0, 0}, 
          actrualdist = sqrt(pow((stick_obs[0]-translation[0]),2)
                               +pow((stick_obs[1]-translation[1]),2));
    if(mindepth > 10) printf("no obstacle ahead\n");
    else
    {
        aver[0] = sum[0] / cnt;
        printf("actrual dist %fm, calculate dist %fm, error %f.\n", actrualdist, mindist, (actrualdist-mindist));
        printf("actrual location (%f,%f), calculate location (%f,%f)\n", stick_obs[0], stick_obs[1], obs[0], obs[1]);
        //printf("average distance is %f.\n", aver[0]);
    }
    //totxt << ros::Time::now() << " " << mindepth << " " << aver << std::endl;
}

//initial subscriber
void subinit(ros::NodeHandle *_n)
{
    uav_sub = _n->subscribe("/vicon/uav_stereo02/uav_stereo02",1, viconCallback);
    depth_sub = _n->subscribe("/camera/depth/image", 1, depthCallback);
    stick_sub = _n->subscribe("/vicon/obstacle_stick/obstacle_stick",1, stickCallback);
    pcl_pub = _n->advertise<sensor_msgs::PointCloud2>("/my/pointcloud",1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rgbd_distance_test");
    ros::NodeHandle n;
    totxt.open("/home/xuewuyang/catkin_ws/src/rgbd_distance_test/result/0");
    subinit(&n);
    printf("start!\n");
    ros::Rate loop_rate(30);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    totxt.close();
    printf("exit!");
    return 1;
}
