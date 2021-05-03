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
                subscriberRGBTopic,
                rmw_qos_profile_sensor_data);

        subscriberDepth = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image >>(
                shared_ptr<rclcpp::Node>(this),
                subscriberDepthTopic,
                rmw_qos_profile_sensor_data);

        syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy >>(
                approximate_sync_policy(10),
                *subscriberRGB,
                *subscriberDepth);

        syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBDImages, this);

        publisherPoseStamped = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                publisherPoseStampedTopic, 10);
        publisherCameraPath = this->create_publisher<nav_msgs::msg::Path>(publisherCameraPathTopic, 10);

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

            transformPose.orientation.w = quaternion.getW();
            transformPose.orientation.x = quaternion.getX();
            transformPose.orientation.y = quaternion.getY();
            transformPose.orientation.z = quaternion.getZ();

            transformPose.position.x = vector3.getX();
            transformPose.position.y = vector3.getY();
            transformPose.position.z = vector3.getZ();

            double roll, pitch, yaw;
            matrix3X3.getRPY(roll, pitch, yaw);  // Get the roll, pitch and yaw and pass it to the matrix.

            if (roll == 0 || pitch == 0 || yaw == 0) return;

            // Establish the header for pose message.
            std_msgs::msg::Header header;
            header.frame_id = subscriberRGBTopic;
            header.stamp = this->get_clock()->now();

            geometry_msgs::msg::PoseStamped poseStampedMessage;
            poseStampedMessage.header = header;
            poseStampedMessage.pose = transformPose;

            nav_msgs::msg::Path cameraPath;
            cameraPath.header = header;
            cameraPath.poses.push_back(poseStampedMessage);
            publisherCameraPath->publish(cameraPath);  //Camera trajectory
            if (isKeyFrame) publisherPoseStamped->publish(poseStampedMessage);  //orbSlamCameraPose pose information
        } else {
            cout << "Twc is empty ..." << endl;
        }
    }

    ORB_SLAM2::System *m_SLAM;

    cv_bridge::CvImageConstPtr cv_pointerRGB;
    cv_bridge::CvImageConstPtr cv_pointerDepth;

    // Topics
    std::string subscriberRGBTopic = "camera/image_raw";
    std::string subscriberDepthTopic = "camera/depth/image_raw";
    std::string publisherPoseStampedTopic = "/stamped_pose";
    std::string publisherCameraPathTopic = "/camera_path";

    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberRGB;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriberDepth;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    // Publishers
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