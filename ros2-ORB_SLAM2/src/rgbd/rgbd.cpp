//
// Created by Alex Musgrove on 26/03/2021.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Tracking.h"

class RgbdSlamNode : public rclcpp::Node {
public:
    explicit RgbdSlamNode(ORB_SLAM2::System *pSLAM)
            : Node("orbslam"),
              m_SLAM(pSLAM) {

        //Handling the RGB and Depth subscribers.
        subscriberRGB = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image >>(
                shared_ptr<rclcpp::Node>(this),
                RGBTopic,
                rmw_qos_profile_sensor_data);

        subscriberDepth = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image >>(
                shared_ptr<rclcpp::Node>(this),
                depthTopic,
                rmw_qos_profile_sensor_data);

        syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy >>(
                approximate_sync_policy(10),
                *subscriberRGB,
                *subscriberDepth);

        syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBDImages, this);


        publisherRGBImage = this->create_publisher<sensor_msgs::msg::Image>(
                processedRGBTopic, rclcpp::SensorDataQoS());
        publisherDepthImage = this->create_publisher<sensor_msgs::msg::Image>(
                processedDepthTopic, rclcpp::SensorDataQoS());
        publisherPoseStamped = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                poseStampedTopic, rclcpp::SensorDataQoS());
        publisherCameraPath = this->create_publisher<nav_msgs::msg::Path>(
                cameraPathTopic, rclcpp::SensorDataQoS());

    }

    ~RgbdSlamNode() override {
        // Stop all threads
        m_SLAM->Shutdown();

        // Save camera trajectory
        m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
            sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabRGBDImages(const sensor_msgs::msg::Image::SharedPtr &messageRGB,
                        const sensor_msgs::msg::Image::SharedPtr &messageDepth) {
        // Copy the ros rgb image message to cv::Mat.
        try {
            cv_pointerRGB = cv_bridge::toCvShare(messageRGB);
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Copy the ros depth image message to cv::Mat.
        try {
            cv_pointerDepth = cv_bridge::toCvShare(messageDepth);
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        bool isKeyFrame = true;
        cv::Mat orbSlamCameraPose = m_SLAM->TrackRGBD(cv_pointerRGB->image,
                                                      cv_pointerDepth->image,
                                                      messageRGB->header.stamp.sec,
                                                      isKeyFrame);

        if (!orbSlamCameraPose.empty()) {
            //cv::Mat Twc =orbSlamCameraPose.inv();
            //cv::Mat TWC=orbslam->mpTracker->mCurrentFrame.mTcw.inv();
            cv::Mat RWC = orbSlamCameraPose.rowRange(0, 3).colRange(0, 3);
            cv::Mat TWC = orbSlamCameraPose.rowRange(0, 3).col(3);

            tf2::Matrix3x3 matrix3X3(RWC.at<float>(0, 0), RWC.at<float>(0, 1), RWC.at<float>(0, 2),
                                     RWC.at<float>(1, 0), RWC.at<float>(1, 1), RWC.at<float>(1, 2),
                                     RWC.at<float>(2, 0), RWC.at<float>(2, 1), RWC.at<float>(2, 2));
            tf2::Vector3 vector3(TWC.at<float>(0), TWC.at<float>(1), TWC.at<float>(2));

            tf2::Quaternion quaternion;
            matrix3X3.getRotation(quaternion); // Get matrix into quaternion form.

            geometry_msgs::msg::Pose transformPose;

            // Set the orientation of the current keyframe transform.
            transformPose.orientation.w = quaternion.getW();
            transformPose.orientation.x = quaternion.getX();
            transformPose.orientation.y = quaternion.getY();
            transformPose.orientation.z = quaternion.getZ();

            // Set the position of the current keyframe transform.
            transformPose.position.x = vector3.getX();
            transformPose.position.y = vector3.getY();
            transformPose.position.z = vector3.getZ();

            // Get the roll, pitch and yaw from the given orbslam's produced quaternion.
            double roll, pitch, yaw;
            matrix3X3.getRPY(roll, pitch, yaw);  // Get the roll, pitch and yaw and pass it to the matrix.

            if (roll == 0 || pitch == 0 || yaw == 0) return;

            // Set the time stamp for the new messages.
            builtin_interfaces::msg::Time timeStamp = this->get_clock()->now();

            // Establish the poseHeader for pose message.
            std_msgs::msg::Header poseHeader;
            poseHeader.frame_id = poseStampedTopic;
            poseHeader.stamp = timeStamp;

            // Set the header and the content of the pose message.
            geometry_msgs::msg::PoseStamped poseStampedMessage;
            poseStampedMessage.header = poseHeader;
            poseStampedMessage.pose = transformPose;

            // Establish the cameraHeader for the camera path message.
            std_msgs::msg::Header cameraHeader;
            cameraHeader.frame_id = cameraPathTopic;
            cameraHeader.stamp = timeStamp;

            // Set the camera message.
            nav_msgs::msg::Path cameraPath;
            cameraPath.header = cameraHeader;
            cameraPath.poses.push_back(poseStampedMessage);

            // Establish the depthHeader for depth message.
            std_msgs::msg::Header depthHeader;
            depthHeader.frame_id = processedDepthTopic;
            depthHeader.stamp = timeStamp;

            // Set the depth message.
            sensor_msgs::msg::Image depthImage = *messageDepth;
            depthImage.header = depthHeader;

            // Establish the rgbHeader for rgb image.
            std_msgs::msg::Header rgbHeader;
            rgbHeader.frame_id = processedRGBTopic;
            rgbHeader.stamp = timeStamp;

            // Set the RGB message.
            sensor_msgs::msg::Image rgbImage = *messageRGB;
            rgbImage.header = rgbHeader;

            publisherCameraPath->publish(cameraPath);  //Camera trajectory.
            if (isKeyFrame) publisherPoseStamped->publish(poseStampedMessage);  //orbSlamCameraPose pose information
            publisherRGBImage->publish(rgbImage);
            publisherDepthImage->publish(depthImage);

        } else {
            cout << "No new keyframe pose has been established." << endl;
        }
    }

    ORB_SLAM2::System *m_SLAM;

    cv_bridge::CvImageConstPtr cv_pointerRGB;
    cv_bridge::CvImageConstPtr cv_pointerDepth;

    // Topics
    std::string RGBTopic = "camera/image_raw";
    std::string depthTopic = "camera/depth/image_raw";
    std::string processedRGBTopic = "orbslam/image_raw";
    std::string processedDepthTopic = "orbslam/depth/image_raw";
    std::string poseStampedTopic = "/stamped_pose";
    std::string cameraPathTopic = "/camera_path";

    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberRGB;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberDepth;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherRGBImage;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherDepthImage;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherPoseStamped;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherCameraPath;
};

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

    auto node = std::make_shared<RgbdSlamNode>(&SLAM);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}