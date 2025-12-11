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

    // ----------- Parameters ---------------
    std::string topic_;
    std::string img_prefix_;
    std::string directory_;

    // ----------- States -------------
    int img_cnt_;
    bool need_img_;

public:
    explicit Capture(int argc, char *argv[])
        : Node("tb3_yolo_capture")
    {
        // Parameters
        this->declare_parameter<std::string>("topic", "/camera/image_raw/compressed");
        this->topic_ = this->get_parameter("topic").get_value<std::string>();

        this->declare_parameter<std::string>("img_prefix", "img");
        this->img_prefix_ = this->get_parameter("img_prefix").get_value<std::string>();

        this->declare_parameter<std::string>("directory", "train_yolo/datasets/traffic_signs/images");
        this->directory_ = this->get_parameter("directory").get_value<std::string>();

        // States
        this->img_cnt_ = 0;
        this->need_img_ = false;

        // Process arguments
        if (argc >= 2)
        {
            this->img_prefix_ = std::string(argv[1]);
            this->set_parameter(rclcpp::Parameter("img_prefix", this->img_prefix_));
        }
        if (argc >= 3)
        {
            this->img_cnt_ = std::atoi(argv[2]);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), 
            "Image prefix is " << this->img_prefix_);
        RCLCPP_INFO_STREAM(this->get_logger(), 
            "Image number starts from " << this->img_cnt_);

        // Handles
        this->cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions options;
        options.callback_group = this->cbg_;

        rclcpp::QoS qos(2);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        this->sub_img_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            this->topic_, qos, std::bind(&Capture::callbackImage_, this, std::placeholders::_1), options);

        RCLCPP_INFO_STREAM(this->get_logger(), "Press [SPACE] to take picture, press [CTRL+C] to exit");
        timer_ = this->create_wall_timer(
            0.05s,
            std::bind(&Capture::callbackTimer_, this), this->cbg_);
    }


private:
    void callbackImage_(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg)
    {
        this->img_ = cv_bridge::toCvCopy(msg);
    }


    /** The function that is run at regular intervals */
    void callbackTimer_()
    {
        if (this->need_img_)
        {
            if (this->img_ == nullptr)
                return;

            this->need_img_ = false;

            ++this->img_cnt_;

            // get file name
            std::filesystem::path fdir(this->directory_);
            std::stringstream ss;
            ss << this->img_prefix_ 
                << std::setw(3) << std::setfill('0') 
                << this->img_cnt_ << ".jpg";
            std::filesystem::path fname(ss.str());

            // create directory if it does not exist
            std::filesystem::path fpath = fdir / fname;
            std::filesystem::path fpath_parent = fpath.parent_path();
            if (!std::filesystem::exists(fpath_parent))
            {
                RCLCPP_INFO_STREAM(this->get_logger(),
                     "Creating directory " << fpath_parent.c_str());
                std::filesystem::create_directory(fpath_parent);
            }

            cv::imwrite(fpath.c_str(), this->img_->image);
            RCLCPP_INFO_STREAM(this->get_logger(), 
                "Created image: " 
                << fname.c_str() 
                << ". Press [SPACE] to take picture, press [CTRL+C] to exit."
            );
        }

        char c;
        getch(c);

        if (c == ' ')
        {
            this->need_img_ = true; // wait until next callback so the next image is used.
        }
    };

    bool getch(char &c)
    {
        c = 0;
        bool error = false;

        // Get old termios settings
        termios oldt;
        if (tcgetattr(STDIN_FILENO, &oldt) < 0)
            return true; // perror("tcsetattr()");

        // Put new termios settings (non canonical, polling read).
        termios newt = oldt;
        newt.c_lflag &= ~ICANON;
        newt.c_lflag &= ~ECHO;
        newt.c_cc[VMIN] = 0; // MIN == 0 required to read with timeout
        newt.c_cc[VTIME] = 0; // TIME > 0 is the number of tenths of seconds to read before timing out
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0)
            error = true; // perror("tcsetattr ICANON");
        
        // Read keyboard
        if (read(0, &c, 1) < 0)
            error = true;

        // Set back the old termios settings
        if (tcsetattr(STDIN_FILENO, TCSANOW, &oldt) < 0)
            error = true; // perror("tcsetattr ~ICANON");
        return error;
    }
};

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Capture>(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}