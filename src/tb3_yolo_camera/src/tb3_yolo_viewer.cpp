#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <termios.h>

#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"


using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"


class Capture : public rclcpp::Node
{

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img_;
    cv_bridge::CvImagePtr img_;
    rclcpp::TimerBase::SharedPtr timer_; // contains the timer that runs the main looping function at regular intervals.
    rclcpp::CallbackGroup::SharedPtr cbg_;

    // std::shared_ptr<image_transport::ImageTransport> it_; 
    // image_transport::Subscriber it_sub_;

    // ----------- Parameters ---------------
    std::string topic_;
    std::string window_;
    std::string img_prefix_;

    // ----------- States -------------
    bool need_capture_;
    bool has_img_;
    int img_cnt_;

public:
    explicit Capture()
        : Node("tb3_yolo_viewer")
    {
        // Parameters
        this->declare_parameter<std::string>("topic", "/camera/image_raw/compressed");
        this->topic_ = this->get_parameter("topic").get_value<std::string>();

        this->declare_parameter<std::string>("window", "camera");
        this->window_ = this->get_parameter("window").get_value<std::string>();

        this->sub_img_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            this->topic_, 1, std::bind(&Capture::callbackImage_, this, std::placeholders::_1));
    }

private:
    void callbackImage_(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg)
    {
        try
        {
            this->img_ = cv_bridge::toCvCopy(msg);
            cv::imshow(this->window_, this->img_->image);
            cv::waitKey(10);
        }
        catch(const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", this->img_->encoding.c_str());
        }
    }
};

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Capture>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}