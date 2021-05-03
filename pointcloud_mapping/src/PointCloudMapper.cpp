//
// Created by Alex Musgrove on 27/04/2021.
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
#include <boost/concept_check.hpp>
// PCL dependencies
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
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
    PointCloudMapper()
            : Node("point_cloud_mapper") {

        //Handling the RGB and Depth subscribers.
        subImageColor = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image >>(
                        // shared_ptr<rclcpp::Node>(this),
                        this,
                        topicColor,
                        rmw_qos_profile_sensor_data);

        subImageDepth = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image >>(
                        this,
                        topicDepth,
                        rmw_qos_profile_sensor_data);
        tcw_sub = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(

                )

        syncApproximate = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy >>(
                ApproximateSyncPolicy(10),
                *subImageColor,
                *subImageDepth,
                *tcw_sub);

        subscriberDepth = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image >>(
                shared_ptr<rclcpp::Node>(this),
                topicDepth,
                rmw_qos_profile_sensor_data);

    }

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberDepth;

    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    bool mbuseExact;
    bool mbuseCompressed = false;
    size_t queueSize = 10;
    size_t lastKeyFrameSize = 0;
    size_t mGlobalPointCloudID = 0;
    size_t mLastGlobalPointCloudID = 0;

    // Topics
    std::string topicColor = "camera/image_raw";
    std::string topicDepth = "camera/depth/image_raw";
    std::string topicTcw = "/stamped_pose";
    std::string publisherCameraPathTopic = "/camera_path";

    // Camera alignment
    float mresolution = 0.04;
    float mDepthMapFactor = 1000.0;
    float mcx = 515.2888, mcy = 517.6610, mfx = 317.9098, mfy = 241.5734;

    void insertKeyFrame(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3d &T) {
        unique_lock <mutex> lck(keyframeMutex);

        mvGlobalPointCloudsPose.push_back(T);
        colorImgs.push_back(color.clone());
        depthImgs.push_back(depth.clone());


        //mLastGlobalPointCloudID=mGlobalPointCloudID;
        mGlobalPointCloudID++;
        mbKeyFrameUpdate = true;

        cout << GREEN << "receive a keyframe, id = " << mGlobalPointCloudID << WHITE << endl;
    };

    void viewer();

    void getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &outputMap);

    void reset();

    void shutdown();

    void callback(const sensor_msgs::msg::Image msgRGB,
                  const sensor_msgs::msg::Image msgD, const geometry_msgs::msg::PoseStamped tcw);

    void callback_pointcloud(const sensor_msgs::msg::Image msgRGB,
                             const sensor_msgs::msg::Image msgD, const geometry_msgs::msg::PoseStamped tcw);
private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subImageColor;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subImageDepth;

protected:

    pcl::VoxelGrid<PointT> voxel; //点云显示精度
    //float mDepthMapFactor =1; //深度图尺度因子
    size_t lastKeyframeSize = 0; //
    size_t mGlobalPointCloudID = 0; //点云ID
    size_t mLastGlobalPointCloudID = 0;

    // data to generate point clouds
    std::vector <cv::Mat> colorImgs, depthImgs;   //image buffer
    cv::Mat depthImg, colorImg, mpose;
    std::vector <PointCloud> mvGlobalPointClouds; //存储关键帧对应的点云序列
    std::vector <Eigen::Isometry3d> mvGlobalPointCloudsPose;

    PointCloud::Ptr globalMap, localMap;

    bool shutDownFlag = false; // 程序退出标志位
    std::mutex shutDownMutex;

    bool mbKeyFrameUpdate = false;        //有新的关键帧插入
    std::mutex keyframeMutex;
    std::mutex keyFrameUpdateMutex;
    std::mutex deletekeyframeMutex;

    typedef message_filters::sync_policies::ExactTime <
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    geometry_msgs::msg::PoseStamped> ExactSyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    geometry_msgs::msg::PoseStamped> ApproximateSyncPolicy;

    image_transport::ImageTransport it;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> tcw_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_sub;

    message_filters::Synchronizer <ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer <ApproximateSyncPolicy> *syncApproximate;

    PointCloud::Ptr generatePointCloud(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3d &T);

    void compressPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, std::stringstream &compressedData);

    void depressPointCloud(std::stringstream &compressedData, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudOut);

    Eigen::Matrix4f cvMat2Eigen(const cv::Mat &cvT);

    void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);


};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudMapper>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}