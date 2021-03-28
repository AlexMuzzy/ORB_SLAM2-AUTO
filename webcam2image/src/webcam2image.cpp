#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui/highgui.hpp"

using namespace std::chrono_literals;

/* Source code: https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp 
   was used as inspiration for this file.*/

class webcamToImage : public rclcpp::Node
{
public:
    webcamToImage()
        : Node("webcam2image"),
          publish_number_(1u)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        initialize();
        timer_ = this->create_wall_timer(
            webcamToImage::calculate_fps(30.0),
            std::bind(&webcamToImage::timer_callback, this));
    }

private:
    // Utility method to calculate running rate.
    std::chrono::milliseconds calculate_fps(double fps_)
    {
        return std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));
    }

    // Initial Method, executes on start.
    void initialize()
    {
        std::vector<int> dimensions{640, 480};

        cap_.open(0);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, dimensions[0]);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, dimensions[1]);

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
            throw std::runtime_error("Could not open video stream");
        }
    }

    // CALLBACK FUNCTION, executes every frame.
    void timer_callback()
    {
        cv::Mat frame;
        auto message = std::make_unique<sensor_msgs::msg::Image>();

        cap_ >> frame;
        convert_matrix_to_message(frame, *message);
        cv::Mat flipped_frame;
        if (publish_number_ % 3 == 0)
        {
            cv::flip(frame, flipped_frame, 1);
            cv::imshow("webcamToImage", flipped_frame);
        }
        else if (publish_number_ % 2 == 0)
        {
            cv::flip(frame, flipped_frame, -1);
            cv::imshow("webcamToImage", flipped_frame);
        }
        else
        {
            cv::imshow("webcamToImage", frame);
        }
        cv::waitKey(1); // Necessary from cv::imshow to work.
        publisher_->publish(*message);
        RCLCPP_INFO(get_logger(), "Publishing image %zd", publish_number_++);
    }

    // OpenCV Matrix encoding to sensor_msgs/Image format.
    std::string mat_type2encoding(int mat_type)
    {
        switch (mat_type)
        {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    void convert_matrix_to_message(const cv::Mat &frame, sensor_msgs::msg::Image &msg)
    {
        // copy cv information into ros message
        msg.height = frame.rows;
        msg.width = frame.cols;
        msg.encoding = mat_type2encoding(frame.type());
        msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        size_t size = frame.step * frame.rows;
        msg.data.resize(size);
        memcpy(&msg.data[0], frame.data, size);
        msg.header.frame_id = "camera_frame";
        msg.header.stamp = this->now();
    }

    rclcpp::TimerBase::SharedPtr timer_;                              // Timed callback attribute.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; // Publish image data to ROS network.
    cv::VideoCapture cap_;                                            // VideoCapture Object to get webcam stream.
    size_t publish_number_;                                           // Current frame number.
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<webcamToImage>());
    rclcpp::shutdown();
    return 0;
}