#include "maze_lidar_sensor/maze_lidar_sensor.hpp"

MazeLidarSensor::MazeLidarSensor()
    : Node("maze_lidar_sensor"),
      exit_number(0),
      right_wall(true),
      current_state(State::FORWARD),
      current_yaw(0.0),
      target_yaw(0.0),
      is_turning(false),
      prev_error(0.0),
      integral(0.0),
      last_turn_direction(NONE),
      left_turn_count(0),
      forward_count(0),
      last_control_time(this->now())
{
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&MazeLidarSensor::scan_callback, this, std::placeholders::_1));

    exit_num_sub = this->create_subscription<std_msgs::msg::Int32>(
        "/turtlebot3_status_manager/number", 10,
        std::bind(&MazeLidarSensor::exit_number_callback, this, std::placeholders::_1));

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10,
        std::bind(&MazeLidarSensor::imu_callback, this, std::placeholders::_1));
}

void MazeLidarSensor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 로우패스 필터 적용
    current_yaw = (1.0 - imu_filter_alpha) * current_yaw + imu_filter_alpha * yaw;

    if (is_turning)
    {
        control_turning();
    }
}

void MazeLidarSensor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (is_turning)
        return;

    if (current_state == State::FORWARD)
    {
        if (last_turn_direction == AFTER_TURN)
        { // 회전 직후라면
            if (check_wall(msg, 0))
            { // 전방 벽 체크
                stop_robot();
                last_turn_direction = NONE; // 초기화
                current_state = State::CHECKING;
                return;
            }
            forward_count++;
            if (forward_count > 20)
            { // 약 2초 동안 전진
                forward_count = 0;
                last_turn_direction = NONE;
            }
            move_forward();
            return;
        }

        if (right_wall)
        { // 우수법일 때
            if (!check_wall(msg, 270))
            { // 오른쪽이 비어있으면
                stop_robot();
                start_turn(-M_PI / 2); // 우회전
                last_turn_direction = RIGHT;
                left_turn_count = 0;
                current_state = State::CHECKING;
                return;
            }
        }
        else
        { // 좌수법일 때
            if (!check_wall(msg, 90))
            { // 왼쪽이 비어있으면
                stop_robot();
                start_turn(M_PI / 2); // 좌회전
                last_turn_direction = LEFT;
                left_turn_count = 0;
                current_state = State::CHECKING;
                return;
            }
        }

        if (check_wall(msg, 0))
        { // 전방에 벽이 있을 때
            stop_robot();
            current_state = State::CHECKING;
        }
        else
        {
            move_forward();
        }
    }

    switch (current_state)
    {
    case State::FORWARD:
        if (check_wall(msg, 0))
        { // 전방에 벽이 있을 때
            stop_robot();
            current_state = State::CHECKING;
        }
        else
        {
            move_forward();
        }
        break;

    case State::CHECKING:
        if (right_wall)
        { // 우수법
            if (!check_wall(msg, 270))
            {                          // 오른쪽에 벽이 없을 때
                start_turn(-M_PI / 2); // 우회전
                last_turn_direction = RIGHT;
                left_turn_count = 0; // 왼쪽 회전 횟수 초기화
            }
            else if (check_wall(msg, 0))
            {                         // 전방에 벽이 있을 때
                start_turn(M_PI / 2); // 좌회전
                last_turn_direction = LEFT;
                left_turn_count = 1; // 왼쪽 회전 횟수 초기화
            }
            else
            {
                current_state = State::FORWARD;
                last_turn_direction = NONE;
                left_turn_count = 0;
            }
        }
        else
        { // 좌수법

            if (!check_wall(msg, 90))
            {                         // 왼쪽에 벽이 없을 때
                start_turn(M_PI / 2); // 좌회전
                last_turn_direction = LEFT;
                left_turn_count = 0;
            }
            else if (check_wall(msg, 0))
            {                          // 전방에 벽이 있을 때
                start_turn(-M_PI / 2); // 우회전
                last_turn_direction = RIGHT;
                left_turn_count = 1;
            }
            else
            {
                current_state = State::FORWARD;
                last_turn_direction = NONE;
                left_turn_count = 0;
            }
        }
        break;

    case State::TURNING:
        // 회전 중에는 처리하지 않음
        break;
    }
}

void MazeLidarSensor::exit_number_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    exit_number = msg->data;
    right_wall = (exit_number == 2 || exit_number == 3);
}

void MazeLidarSensor::start_turn(double angle)
{
    // 현재 움직임 정지
    stop_robot();

    // 약간의 딜레이
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    is_turning = true;
    target_yaw = normalize_angle(current_yaw + angle);
    prev_error = 0.0;
    integral = 0.0;
    last_control_time = this->now();

    RCLCPP_INFO(this->get_logger(),
                "Starting turn: current=%.2f, target=%.2f",
                current_yaw, target_yaw);
}

void MazeLidarSensor::control_turning()
{
    double error = normalize_angle(target_yaw - current_yaw);

    // 시간 기반 제어
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_control_time).seconds();

    if (dt > 0.1)
    {
        integral = 0;
        prev_error = error;
        last_control_time = current_time;
        return;
    }

    // PID 제어 그대로
    integral = clamp(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
    double derivative = (error - prev_error) / dt;

    double angular_velocity = Kp * error + Ki * integral + Kd * derivative;

    // 각속도가 너무 작을 때 방지하려고 약간 증가하는 부분
    if (std::abs(angular_velocity) < 0.1 && std::abs(error) > ANGLE_THRESHOLD)
    {
        angular_velocity = (error > 0) ? 0.1 : -0.1;
    }

    angular_velocity = clamp(angular_velocity, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    prev_error = error;
    last_control_time = current_time;//시간 업데이트하는 부분

    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = angular_velocity;
    cmd_vel_pub->publish(twist);

    static int stable_count = 0;

    // 완화된 정지 조건
    if (std::abs(error) < ANGLE_THRESHOLD * 1.5)
    { // 허용 오차 증가
        stable_count++;

        // 일정 시간 동안 허용 오차 내에 있으면 정지
        if (stable_count > 10)
        {
            is_turning = false;
            stop_robot();
            current_state = State::FORWARD;
            last_turn_direction = AFTER_TURN; // 회전 완료 후 AFTER_TURN 상태로
            forward_count = 0;                // 전진 카운트 초기화
            stable_count = 0;
            RCLCPP_INFO(this->get_logger(), "Turn completed with error: %.3f", error);
        }
    }
    else
    {
        stable_count = 0;
    }

}

void MazeLidarSensor::move_forward()
{
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = MOVE_SPEED;
    cmd_vel_pub->publish(twist);
}

void MazeLidarSensor::stop_robot()
{
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub->publish(twist);
}

bool MazeLidarSensor::check_wall(const sensor_msgs::msg::LaserScan::SharedPtr msg, int angle)
{
    for (int i = -30; i <= 30; i++)
    {
        int idx = ((angle + i) + 360) % 360;
        if (msg->ranges[idx] < WALL_THRESHOLD &&
            msg->ranges[idx] > msg->range_min)
        {
            return true;
        }
    }
    return false;
}

double MazeLidarSensor::normalize_angle(double angle)
{
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0)
        angle += 2.0 * M_PI;
    return angle - M_PI;
}