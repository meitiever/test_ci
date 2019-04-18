/**
 * @file PointCloudProcess.cc
 * @author your name (wsh@sineva.com.cn)
 * @brief 
 * @version 0.1
 * @date 2019-04-09
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <stdio.h>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

ros::Publisher kPointCloudPublish;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr kPointCloud;

const std::string kCoordinateType = "world";
double fx, fy, cx, cy;

Eigen::Affine3d messageTransformToEigen(const nav_msgs::Odometry &msg)
{
    return Eigen::Affine3d(
        Eigen::Translation3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) *
        Eigen::Quaterniond(msg.pose.pose.orientation.w,
                           msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z));
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr generatePointCloud(const cv::Mat &color, const cv::Mat &depth)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for (int m = 0; m < depth.rows; m += 2)
    {
        for (int n = 0; n < depth.cols; n += 2)
        {
            float d = depth.ptr<float>(m)[n];
            // std::cout  << "depth: " << d << std::endl;
            // if (d > 0.2 && d < 15)
            {
                pcl::PointXYZRGBA p;
                p.z = d;
                p.x = (n - cx) * p.z / fx;
                p.y = (m - cy) * p.z / fy;

                if (color.channels() == 3)
                {
                    p.r = color.ptr<uchar>(m)[n * 3];
                    p.g = color.ptr<uchar>(m)[n * 3 + 1];
                    p.b = color.ptr<uchar>(m)[n * 3 + 2];
                }
                else if (color.channels() == 4)
                {
                    p.r = color.ptr<uchar>(m)[n * 4];
                    p.g = color.ptr<uchar>(m)[n * 4 + 1];
                    p.b = color.ptr<uchar>(m)[n * 4 + 2];
                    p.a = color.ptr<uchar>(m)[n * 4 + 3];
                }
                tempPointCloud->points.push_back(p);
            }
        }
    }
    tempPointCloud->is_dense = false;
    pcl::visualization::CloudViewer viewer("Viewer");
    viewer.showCloud(tempPointCloud);
    while (!viewer.wasStopped())
    {
    }
    std::cout << "temp pointcloud size: " << tempPointCloud->points.size() << std::endl;
    return tempPointCloud;
}

class dataGrabber
{
  public:
  dataGrabber(){}
    void dataCatcher(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
};

void dataGrabber::dataCatcher(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    generatePointCloud(cv_ptrRGB->image,cv_ptrD->image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PointCloudProcess");
    ros::NodeHandle nh;

    std::string topicDepth, topicColor, parameterPath;
    nh.getParam("topicDepth", topicDepth);
    nh.getParam("topicColor", topicColor);
    nh.getParam("parameterPath", parameterPath);

    cv::FileStorage fSettings(parameterPath, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        std::cerr << "Failed to open settings file at: " << parameterPath << std::endl;
        exit(-1);
    }
    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
    // fSettings.release();

    kPointCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    message_filters::Subscriber<sensor_msgs::Image> rgbImageSub(nh, topicColor, 1);
    message_filters::Subscriber<sensor_msgs::Image> depthImageSub(nh, topicDepth, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPol;
    message_filters::Synchronizer<SyncPol> sync(SyncPol(10), rgbImageSub, depthImageSub);
    dataGrabber data;
    sync.registerCallback(boost::bind(&dataGrabber::dataCatcher, &data, _1, _2));

    ros::spin();

    return 0;
}
