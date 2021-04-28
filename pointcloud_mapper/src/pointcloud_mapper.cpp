//
// Created by alexmuzzy on 27/04/2021.
//
// C++ standard libraries
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
// PCL dependencies
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV
#include <opencv2/opencv.hpp>
// ROS2 Foxy
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

class PointCloudMapper : public rclcpp::Node {
public:
    // ROS2 Node Constructor.
    PointCloudMapper()
            : Node("point_cloud_mapper") {
        subscriberRGBImage = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image >>(
                std::shared_ptr<rclcpp::Node>(this),
                subscriberRGBTopic,
                rmw_qos_profile_sensor_data);

        subscriberDepthImage = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image >>(
                std::shared_ptr<rclcpp::Node>(this),
                subscriberDepthTopic,
                rmw_qos_profile_sensor_data);

        subscriberPoseStamped = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped >>(
                std::shared_ptr<rclcpp::Node>(this),
                "/camera/depth/image_raw",
                rmw_qos_profile_sensor_data);

        syncApproximate = std::make_shared<message_filters::Synchronizer<approximateSyncPolicy >>(
                approximateSyncPolicy(10),
                *subscriberRGBImage,
                *subscriberDepthImage,
                *subscriberPoseStamped);

        voxel.setLeafSize(resolution, resolution, resolution);

        syncApproximate->registerCallback(&PointCloudMapper::callback, this);


        publisherLocalPointCloud =
                this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        publisherGlobalPointCloudTopic, 1);

        publisherGlobalPointCloud =
                this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        publisherLocalPointCloudTopic, 10);

        // Initialize PCL viewer.
        FUpdate = false;
        KFUpdate = false;
        LoopCloserUpdate = false;
        pclViewer = pcl::visualization::CloudViewer::Ptr(
                new pcl::visualization::CloudViewer ("ORB SLAM2 Viewer"));

        // Log all configuration information.
        std::cout << "topicColor: " << subscriberRGBImage << std::endl;
        std::cout << "topicDepth: " << subscriberDepthImage << std::endl;
        std::cout << "topicTcw: " << subscriberPoseStamped << std::endl;
        std::cout << "fx: " << cameraFx << std::endl;
        std::cout << "fy: " << cameraFy << std::endl;
        std::cout << "cx: " << cameraCx << std::endl;
        std::cout << "cy: " << cameraCy << std::endl;
        std::cout << "resolution: " << resolution << std::endl;
        std::cout << "DepthMapFactor: " << depthMapFactor << std::endl;
        std::cout << "queueSize: " << queueSize << std::endl;
    }

    ~PointCloudMapper() override {
        {
            std::unique_lock<std::mutex> lck(shutDownMutex);
            shutDownFlag = true;
        }
        std::string save_path = "/home/crp/resultPointCloudFile.pcd";
        pcl::io::savePCDFile(save_path, *globalMap);
        std::cout << "save pcd files to :  " << save_path << std::endl;
    }

    void insertKeyFrame(cv::Mat &colourImage, cv::Mat &depthImage, Eigen::Isometry3d &T) {
        std::unique_lock<std::mutex> lck(keyframeMutex);

        mvGlobalPointCloudsPose.push_back(T);
        colourImages.push_back(colourImage.clone());
        depthImages.push_back(depthImage.clone());

        globalPointCloudID++;
        boolKeyFrameUpdate = true;

        std::cout << GREEN << "receive a keyframe, id = " << globalPointCloudID << WHITE << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr generatePointCloud(
            cv::Mat &color, cv::Mat &depth, Eigen::Isometry3d &T) {

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now(); // Get the current time.
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>());
        // Given point cloud is null pointer.

        for (int m = 0; m < depth.rows; m += 3) {
            for (int n = 0; n < depth.cols; n += 3) {
                float d = depth.ptr<float>(m)[n] / depthMapFactor;
                if (d < 0.01 || d > 10) continue;
                pcl::PointXYZRGBA pointXyzrgba;
                pointXyzrgba.z = d;
                pointXyzrgba.x = (n - cameraCx) * pointXyzrgba.z / cameraFx;
                pointXyzrgba.y = (m - cameraCy) * pointXyzrgba.z / cameraFy;

                pointXyzrgba.r = color.ptr<uchar>(m)[n * 3];
                pointXyzrgba.g = color.ptr<uchar>(m)[n * 3 + 1];
                pointXyzrgba.b = color.ptr<uchar>(m)[n * 3 + 2];

                temp->points.push_back(pointXyzrgba);
            }
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_voxel_tem(new pcl::PointCloud<pcl::PointXYZRGBA>);
        temp->is_dense = false;
        voxel.setInputCloud(temp);
        voxel.setLeafSize(resolution, resolution, resolution);
        voxel.filter(*cloud_voxel_tem);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::transformPointCloud(*cloud_voxel_tem, *cloud1, T.matrix());

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        std::cout << GREEN << "generate point cloud from  kf-ID:" << lastGlobalPointCloudID << ", size="
                  << cloud1->points.size() << " cost time: " << time_used.count() * 1000 << " ms ." << WHITE
                  << std::endl;
        lastGlobalPointCloudID++;
        return cloud1;
    }

    void updateViewer() {
        KFUpdate = false;
        {
            std::unique_lock<std::mutex> lck(keyframeMutex);
            N = mvGlobalPointCloudsPose.size();
            KFUpdate = boolKeyFrameUpdate;
            boolKeyFrameUpdate = false;
        }
        if (KFUpdate) {
            for (i = lastKeyframeSize; i < N && i < (lastKeyframeSize + 5); i++) {
                if ((mvGlobalPointCloudsPose.size() != colourImages.size()) ||
                    (mvGlobalPointCloudsPose.size() != depthImages.size()) ||
                    (depthImages.size() != colourImages.size())) {
                    cout << " depthImages.size != colourImages.size()  " << endl;
                    continue;
                }
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tem_cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>());

                cout << "i: " << i << "  mvPosePointClouds.size(): " << mvGlobalPointCloudsPose.size() << endl;
                tem_cloud1 = generatePointCloud(colourImages[i], depthImages[i], mvGlobalPointCloudsPose[i]);

                if (tem_cloud1->empty())
                    continue;

                *globalMap += *tem_cloud1;

                sensor_msgs::msg::PointCloud2 local;
                pcl::toROSMsg(*tem_cloud1, local);
                local.header.stamp = this->get_clock()->now();
                local.header.frame_id = "world";
                publisherLocalPointCloud->publish(local);
            }
        }

        {
            int buff_length = 150;
            if (i > (buff_length + 5)) {
                std::unique_lock<std::mutex> lck(deletekeyframeMutex);
                mvGlobalPointCloudsPose.erase(
                        mvGlobalPointCloudsPose.begin(),
                        mvGlobalPointCloudsPose.begin() + buff_length / 2);

                depthImages.erase(depthImages.begin(), depthImages.begin() + buff_length);
                colourImages.erase(colourImages.begin(), colourImages.begin() + buff_length);

                i = i - buff_length;

                std::cout << RED << "delete keyframe ...." << WHITE << std::endl;
            }
        }

        lastKeyframeSize = i;
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*globalMap, output);
        output.header.stamp = this->get_clock()->now();
        output.header.frame_id = "world";
        publisherGlobalPointCloud->publish(output);
        pclViewer->showCloud(globalMap);
        cout << "show global map, size=" << globalMap->points.size() << endl;
    }

    void callback(const sensor_msgs::msg::Image messageRGBImage,
                  const sensor_msgs::msg::Image messageDepthImage,
                  const geometry_msgs::msg::PoseStamped messagePoseStamped) {

        cv::Mat colourImage, depthImage;
        cv_bridge::CvImagePtr cvImagePtr;

        cvImagePtr = cv_bridge::toCvCopy(messageRGBImage, "rgb8");
        colourImage = cvImagePtr->image;

        cvImagePtr = cv_bridge::toCvCopy(messageDepthImage, messageDepthImage.encoding); //imageDepth->encoding
        cvImagePtr->image.copyTo(depthImage);
        // IR image input
        if (colourImage.type() == CV_16U) {
            cv::Mat tmp;
            colourImage.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, colourImage, CV_GRAY2BGR);
        }
        // 	if(depthImage.type() != CV_16U)
        // 	{
        // 		// cv::Mat tmp;
        // 		depthImage.convertTo(depthImage, CV_16U);
        // 		// cv::cvtColor(tmp, colourImage, CV_GRAY2BGR);
        // 	}

        if (depthImage.type() != CV_32F)
            depthImage.convertTo(depthImage, CV_32F);

        Eigen::Quaterniond q = Eigen::Quaterniond(messagePoseStamped.pose.orientation.w,
                                                  messagePoseStamped.pose.orientation.x,
                                                  messagePoseStamped.pose.orientation.y,
                                                  messagePoseStamped.pose.orientation.z);
        Eigen::AngleAxisd V6(q);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(V6);
        T(0, 3) = messagePoseStamped.pose.position.x;
        T(1, 3) = messagePoseStamped.pose.position.y;
        T(2, 3) = messagePoseStamped.pose.position.z;

        insertKeyFrame(colourImage, depthImage, T);
        updateViewer();
    };

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image,
            geometry_msgs::msg::PoseStamped> approximateSyncPolicy;



    // Topic
    std::string pclViewerName = "ORB SLAM2 Viewer";
    std::string subscriberRGBTopic = "camera/image_raw";
    std::string subscriberDepthTopic = "camera/depth/image_raw";
    std::string publisherPoseStampedTopic = "/stamped_pose";
    std::string publisherGlobalPointCloudTopic = "Global/PointCloudOutput";
    std::string publisherLocalPointCloudTopic = "Local/PointCloudOutput";

    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberRGBImage;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberDepthImage;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> subscriberPoseStamped;
    std::shared_ptr<message_filters::Synchronizer<approximateSyncPolicy>> syncApproximate;

    // Camera Calibration information
    size_t cameraCx = 515.2888;
    size_t cameraFx = 317.9098;
    size_t cameraFy = 517.6610;
    size_t cameraCy = 241.5734;
    float resolution = 0.04;
    float depthMapFactor = 1000.0;
    size_t queueSize = 10;

    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr localMap;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMap;
    pcl::visualization::CloudViewer::Ptr pclViewer;

    size_t lastKeyframeSize = 0;
    size_t globalPointCloudID = 0;

    size_t lastGlobalPointCloudID = 0;


    // Shutdown constants
    bool shutDownFlag = false;
    std::mutex shutDownMutex;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherLocalPointCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherGlobalPointCloud;

    // data to generate point clouds
    std::vector<cv::Mat> colourImages, depthImages;   //image buffer
    cv::Mat depthImage, colourImage, pose;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> mvGlobalPointClouds;
    std::vector<Eigen::Isometry3d> mvGlobalPointCloudsPose;

    bool boolKeyFrameUpdate;
    bool FUpdate;
    bool KFUpdate;
    bool LoopCloserUpdate;

    size_t N = 0;
    size_t i = 0;

    // Mutexes
    std::mutex keyframeMutex;
    std::mutex keyFrameUpdateMutex;
    std::mutex deletekeyframeMutex;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::shared_ptr<PointCloudMapper>());
    rclcpp::shutdown();
    return 0;
}

