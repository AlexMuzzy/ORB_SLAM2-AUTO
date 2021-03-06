//
// Created by Alex Musgrove on 14/03/2021.
//
// C++ standard libraries
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
// ROS2 Foxy
#include "rclcpp/rclcpp.hpp"
// PCL dependencies
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
// OpenCV
#include <opencv2/core/core.hpp>
// ROS2 Dependencies
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

class PointCloudMapper : public rclcpp::Node
{
public:
    // ROS2 Node Constructor.
    PointCloudMapper()
        : Node("point_cloud_mapper")
    {
        subscriberRGBImage = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            std::shared_ptr<rclcpp::Node>(this),
            rgbTopic,
            rmw_qos_profile_sensor_data);

        subscriberDepthImage = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            std::shared_ptr<rclcpp::Node>(this),
            depthTopic,
            rmw_qos_profile_sensor_data);

        subscriberPoseStamped = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
            std::shared_ptr<rclcpp::Node>(this),
            poseStampedTopic,
            rmw_qos_profile_sensor_data);

        syncApproximate = std::make_shared<message_filters::Synchronizer<approximateSyncPolicy>>(
            approximateSyncPolicy(10),
            *subscriberRGBImage,
            *subscriberDepthImage,
            *subscriberPoseStamped);

        syncApproximate->registerCallback(&PointCloudMapper::callback, this);

        publisherLocalPointCloud =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                globalPointCloudTopic, rclcpp::SensorDataQoS());

        publisherGlobalPointCloud =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                localPointCloudTopic, rclcpp::SensorDataQoS());

        // Initialize PCL viewer.
        keyFrameUpdate = false;
        pclViewer = pcl::visualization::CloudViewer::Ptr(
            new pcl::visualization::CloudViewer(pclViewerName));
        globalMap = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

        // Log all configuration information.
        RCLCPP_INFO(this->get_logger(), "colourImageTopic: " + rgbTopic);
        RCLCPP_INFO(this->get_logger(), "depthImageTopic: " + depthTopic);
        RCLCPP_INFO(this->get_logger(), "poseStampedTopic: " + poseStampedTopic);
        RCLCPP_INFO(this->get_logger(), "fx: " + std::to_string(cameraFx));
        RCLCPP_INFO(this->get_logger(), "fy: " + std::to_string(cameraFy));
        RCLCPP_INFO(this->get_logger(), "cx: " + std::to_string(cameraCx));
        RCLCPP_INFO(this->get_logger(), "cy: " + std::to_string(cameraCy));
        RCLCPP_INFO(this->get_logger(), "resolution: " + std::to_string(resolution));
        RCLCPP_INFO(this->get_logger(), "DepthMapFactor: " + std::to_string(depthMapFactor));
        RCLCPP_INFO(this->get_logger(), "queueSize: " + std::to_string(queueSize));
    }

    ~PointCloudMapper() override
    {
        std::string save_path = "/home/alexmuzzy/resultPointCloudFile.pcd";
        pcl::io::savePCDFile(save_path, *globalMap);
        std::cout << "save pcd files to :  " << save_path << std::endl;
        RCLCPP_INFO(this->get_logger(), "PointCloud Mapper shutting down. Saving out to PCD file: " + save_path);
    }

    void insertKeyFrame(cv::Mat &newColourImage, cv::Mat &newDepthImage, Eigen::Isometry3d &transform)
    {
        globalPointCloudsPose.push_back(transform);
        colourImages.push_back(newColourImage.clone());
        depthImages.push_back(newDepthImage.clone());

        globalPointCloudID++;
        keyFrameUpdate = true;

        RCLCPP_INFO(this->get_logger(), "Keyframe received. Keyframe ID: " + std::to_string(globalPointCloudID));
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr generatePointCloud(
        cv::Mat &color, cv::Mat &depth, Eigen::Isometry3d &T)
    {

        std::chrono::steady_clock::time_point timePoint1 = std::chrono::steady_clock::now(); // Get the current time.
        pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
        // Given point cloud is null pointer.

        RCLCPP_INFO(this->get_logger(),
                    "Generating point-cloud. Rows: " + std::to_string(depth.rows) +
                        "Columns: " + std::to_string(depth.cols));
        for (int m = 0; m < depth.rows; m += 3)
        {
            for (int n = 0; n < depth.cols; n += 3)
            {
                float depthPoint = depth.ptr<float>(m)[n] / depthMapFactor;
                RCLCPP_DEBUG(this->get_logger(), "Depth value for " +
                                                     std::to_string(m) + ", " +
                                                     std::to_string(n) + ": " +
                                                     std::to_string(depthPoint));
                if (depthPoint <= minDepth || depthPoint > maxDepth)
                    continue;
                PointT pointXyzrgb;
                pointXyzrgb.z = depthPoint;
                pointXyzrgb.x = ((float)n - cameraCx) * pointXyzrgb.z / cameraFx;
                pointXyzrgb.y = ((float)m - cameraCy) * pointXyzrgb.z / cameraFy;

                pointXyzrgb.r = color.ptr<uchar>(m)[n * 3];
                pointXyzrgb.g = color.ptr<uchar>(m)[n * 3 + 1];
                pointXyzrgb.b = color.ptr<uchar>(m)[n * 3 + 2];

                temp->points.push_back(pointXyzrgb);
            }
        }

        pcl::PointCloud<PointT>::Ptr tempVoxelCloud(new pcl::PointCloud<PointT>);
        temp->is_dense = false;
        voxel.setInputCloud(temp);
        voxel.setLeafSize(resolution, resolution, resolution);
        voxel.filter(*tempVoxelCloud);

        pcl::PointCloud<PointT>::Ptr newCloud(new pcl::PointCloud<PointT>);

        // Inverse the points given in the pointcloud to match the coordinate system given in the PCL viewer.
        pcl::transformPointCloud(*tempVoxelCloud, *newCloud, T.inverse().matrix());

        std::chrono::steady_clock::time_point timePoint2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(timePoint2 - timePoint1);

        RCLCPP_INFO(this->get_logger(),
                    "generate point cloud from Key-Frame ID: " + std::to_string(lastGlobalPointCloudID) +
                        ", Size = " + std::to_string(newCloud->points.size()) +
                        " Time Taken: " + std::to_string(time_used.count() * 1000) + " ms.");

        lastGlobalPointCloudID++;
        return newCloud;
    }

    void updateViewer()
    {
        N = globalPointCloudsPose.size();
        if (keyFrameUpdate)
        {
            for (i = lastKeyframeSize; i < N && i < (lastKeyframeSize + 5); i++)
            {
                if ((globalPointCloudsPose.size() != colourImages.size()) ||
                    (globalPointCloudsPose.size() != depthImages.size()) ||
                    (depthImages.size() != colourImages.size()))
                {
                    RCLCPP_INFO(this->get_logger(), "Size of depth images does not match colour images.");
                    continue;
                }
                localMap = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
                RCLCPP_INFO(
                    this->get_logger(), "i: " + std::to_string(i) + "  mvPosePointClouds.size(): " +
                                            std::to_string(globalPointCloudsPose.size()));

                localMap = generatePointCloud(colourImages[i], depthImages[i], globalPointCloudsPose[i]);

                if (localMap->empty())
                    continue;
                RCLCPP_INFO(this->get_logger(), "Adding pointcloud " + std::to_string(i) + " to global map.");
                *globalMap += *localMap;

                sensor_msgs::msg::PointCloud2 local;
                pcl::toROSMsg(*localMap, local);
                local.header.stamp = this->get_clock()->now();
                local.header.frame_id = "world";
                publisherLocalPointCloud->publish(local);
            }
        }

        int buff_length = 150;
        if ((int)i > (buff_length + 5))
        {
            globalPointCloudsPose.erase(
                    globalPointCloudsPose.begin(),
                    globalPointCloudsPose.begin() + buff_length / 2);

            depthImages.erase(depthImages.begin(), depthImages.begin() + buff_length);
            colourImages.erase(colourImages.begin(), colourImages.begin() + buff_length);

            i = i - buff_length;

            RCLCPP_INFO(this->get_logger(), "delete keyframe(s) ....");
        }

        lastKeyframeSize = i;
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*globalMap, output);
        output.header.stamp = this->get_clock()->now();
        output.header.frame_id = "world";
        publisherGlobalPointCloud->publish(output);
        pclViewer->showCloud(globalMap);
        RCLCPP_INFO(this->get_logger(), "Showing global map, size = " + std::to_string(globalMap->points.size()));
    }

    void callback(const sensor_msgs::msg::Image &messageRGB,
                  const sensor_msgs::msg::Image &messageDepth,
                  const geometry_msgs::msg::PoseStamped &messagePoseStamped)
    {

        cv::Mat colourImage, depthImage;
        cv_bridge::CvImagePtr cvColourImagePtr, cvDepthImagePtr;

        try
        {
            cvColourImagePtr = cv_bridge::toCvCopy(messageRGB);
            colourImage = cvColourImagePtr->image;
            cvDepthImagePtr = cv_bridge::toCvCopy(messageDepth);
            depthImage = cvDepthImagePtr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (colourImage.type() == CV_16U)
        {
            cv::Mat tmp;
            colourImage.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, colourImage, CV_GRAY2BGR);
        }

        if (depthImage.type() != CV_32F)
            depthImage.convertTo(depthImage, CV_32F);

        Eigen::Quaterniond quaternion = Eigen::Quaterniond(messagePoseStamped.pose.orientation.w,
                                                           messagePoseStamped.pose.orientation.x,
                                                           messagePoseStamped.pose.orientation.y,
                                                           messagePoseStamped.pose.orientation.z);
        Eigen::AngleAxisd angleAxis(quaternion);
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(angleAxis);
        transform(0, 3) = messagePoseStamped.pose.position.x;
        transform(1, 3) = messagePoseStamped.pose.position.y;
        transform(2, 3) = messagePoseStamped.pose.position.z;

        insertKeyFrame(colourImage, depthImage, transform);
        updateViewer();
    };

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        geometry_msgs::msg::PoseStamped>
        approximateSyncPolicy;

    typedef pcl::PointXYZRGBA PointT;

    // Topic
    std::string pclViewerName = "ORB SLAM2 Viewer";
    std::string rgbTopic = "orbslam/image_raw";
    std::string depthTopic = "orbslam/depth/image_raw";
    std::string poseStampedTopic = "/stamped_pose";
    std::string globalPointCloudTopic = "Global/PointCloudOutput";
    std::string localPointCloudTopic = "Local/PointCloudOutput";

    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberRGBImage;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberDepthImage;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> subscriberPoseStamped;
    std::shared_ptr<message_filters::Synchronizer<approximateSyncPolicy>> syncApproximate;

    // Camera Calibration information
    float cameraFx = 520.908620;
    float cameraFy = 521.007327;
    float cameraCx = 325.141442;
    float cameraCy = 249.701764;
    float resolution = 0.04;
    float depthMapFactor = 1.0;
    float minDepth = 0.0;
    float maxDepth = 20.0;

    // Queue Buffer Size
    size_t queueSize = 10;

    pcl::VoxelGrid<PointT> voxel;
    pcl::PointCloud<PointT>::Ptr localMap;
    pcl::PointCloud<PointT>::Ptr globalMap;
    pcl::visualization::CloudViewer::Ptr pclViewer;

    size_t lastKeyframeSize = 0;
    size_t globalPointCloudID = 0;

    size_t lastGlobalPointCloudID = 0;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherLocalPointCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherGlobalPointCloud;

    // data to generate point clouds
    std::vector<cv::Mat> colourImages, depthImages;
    std::vector<Eigen::Isometry3d> globalPointCloudsPose;

    bool keyFrameUpdate;

    size_t N = 0;
    size_t i = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMapper>());
    rclcpp::shutdown();
    return 0;
}
