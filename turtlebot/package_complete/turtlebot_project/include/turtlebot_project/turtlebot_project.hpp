// turtlebot_project.hpp
#ifndef TURTLEBOT_PROJECT_HPP
#define TURTLEBOT_PROJECT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>

class TurtlebotProject : public rclcpp::Node {
public:
    TurtlebotProject();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void directionCallback(const std_msgs::msg::String::SharedPtr msg);
    cv::Mat detectYellowLine(const cv::Mat& image);
    cv::Mat detectWhiteLine(const cv::Mat& image);
    cv::Mat detectGreenLine(const cv::Mat& image);
    cv::Mat detectBlueLine(const cv::Mat& image);
    cv::Mat detectRedLine(const cv::Mat& image);
    bool should_stop = false;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direction_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    
    double angular_speed = 0.9;
    double linear_speed = 0.055;
    bool is_accelerated = false;
    std::string current_direction = "BOTH";

    // 파란색 선 카운트를 위한 변수들
    int blue_line_count = 0;
    bool is_blue_line_detected = false;
    bool intersection = false;

    std::queue<geometry_msgs::msg::Twist> delayed_commands;
    const std::size_t delay_frames = 30;
};

#endif