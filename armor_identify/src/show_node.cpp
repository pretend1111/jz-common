#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std;
using namespace cv;

class VideoSubscriber : public rclcpp::Node
{
public:
    VideoSubscriber() : Node("video_subscriber")
    {
        // 创建订阅者，订阅颜色信息
        color_sub_ = this->create_subscription<std_msgs::msg::String>(
            "enemy_color", 10, std::bind(&VideoSubscriber::color_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅数字信息
        number_sub_ = this->create_subscription<std_msgs::msg::String>(
            "enemy_number", 10, std::bind(&VideoSubscriber::number_callback, this, std::placeholders::_1));

        // 打开视频
        cap_ = VideoCapture("/home/pretend/code/dev_ws/src/armor_identify/video/6.mp4");
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file!");
            exit(1);
        }
    }

    void color_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 更新接收到的颜色信息
        enemy_color_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received enemy color: %s", enemy_color_.c_str());
    }

    void number_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 更新接收到的数字信息
        enemy_number_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received enemy number: %s", enemy_number_.c_str());
    }

    void run()
    {
        Mat img;
        while (rclcpp::ok())
        {
            cap_ >> img;
            if (img.empty())
            {
                RCLCPP_INFO(this->get_logger(), "End of video stream.");
                break;
            }

            // 在视频左上角显示敌人颜色
            putText(img, "Color: " + enemy_color_, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);

            // 在视频右上角显示敌人数字
            int baseline = 0;
            Size text_size = getTextSize("Number: " + enemy_number_, FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
            putText(img, "Number: " + enemy_number_, Point(img.cols - text_size.width - 10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);

            imshow("video", img);

            if (waitKey(20) >= 0)
            {
                break;
            }
        }
    }

private:
    VideoCapture cap_;
    string enemy_color_ = "unknown";
    string enemy_number_ = "unknown";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr number_sub_;
};

int main(int argc, char **argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto video_subscriber = std::make_shared<VideoSubscriber>();

    // 启动节点
    video_subscriber->run();

    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}