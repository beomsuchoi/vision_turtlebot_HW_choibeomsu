#ifndef MAZE_LIDAR_SENSOR_HPP
#define MAZE_LIDAR_SENSOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>

enum TurnDirection
{
    NONE,
    LEFT,
    RIGHT,
    AFTER_TURN
};

class MazeLidarSensor : public rclcpp::Node
{
public:
    MazeLidarSensor();

private:
    // Publishers & Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr exit_num_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    // 상태 변수
    int exit_number;
    bool right_wall;
    enum class State
    {
        FORWARD,  // 전진
        CHECKING, // 벽 확인
        TURNING   // 회전
    };
    State current_state;

    TurnDirection last_turn_direction;
    int left_turn_count;
    static const int MAX_LEFT_TURNS = 3;
    int forward_count;

    // IMU 관련 변수
    double current_yaw;
    double target_yaw;
    bool is_turning;

    // PID 제어 변수
    double prev_error;
    double integral;
    rclcpp::Time last_control_time;
    static constexpr double Kp = 0.6;   // 비례
    static constexpr double Ki = 0.005; // 적분
    static constexpr double Kd = 0.2;   // 미분

    // IMU 필터링 관련
    static constexpr double imu_filter_alpha = 0.1; // IMU 가중치

    // 파라미터
    static constexpr double WALL_THRESHOLD = 0.22;   // 벽 감지 거리
    static constexpr double MOVE_SPEED = 0.15;       // 이동 속도
    static constexpr double ANGLE_THRESHOLD = 0.03;  // 각도 오차
    static constexpr double MAX_ANGULAR_SPEED = 0.4; // 최대 회전 속도
    static constexpr double MAX_INTEGRAL = 0.3;      // 적분 최대값

    // Callback 함수
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void exit_number_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // 제어 함수
    void stop_robot();
    void start_turn(double angle);
    void control_turning();
    bool check_wall(const sensor_msgs::msg::LaserScan::SharedPtr msg, int angle);
    double normalize_angle(double angle);

    void move_forward();
    void move_forward_after_turn();

    double clamp(double value, double min, double max)
    {
        return std::max(min, std::min(max, value));
    }
};

#endif