#include "turtlebot_project/turtlebot_project.hpp"

TurtlebotProject::TurtlebotProject() : Node("turtlebot_project")
{
    image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&TurtlebotProject::imageCallback, this, std::placeholders::_1));

    direction_subscriber = this->create_subscription<std_msgs::msg::String>(
        "/turtlebot3_status_manager/direction", 10,
        std::bind(&TurtlebotProject::directionCallback, this, std::placeholders::_1));

    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void TurtlebotProject::directionCallback(const std_msgs::msg::String::SharedPtr msg)
{
    current_direction = msg->data;
    RCLCPP_INFO(this->get_logger(), "Direction changed to: %s", current_direction.c_str());
}

cv::Mat TurtlebotProject::detectYellowLine(const cv::Mat &image)
{
    cv::Mat hsv, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(40, 255, 255);
    cv::inRange(hsv, lower_yellow, upper_yellow, mask);
    return mask;
}

cv::Mat TurtlebotProject::detectWhiteLine(const cv::Mat &image)
{
    cv::Mat hsv, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_white(0, 0, 150);
    cv::Scalar upper_white(180, 60, 255);
    cv::inRange(hsv, lower_white, upper_white, mask);
    return mask;
}

cv::Mat TurtlebotProject::detectGreenLine(const cv::Mat &image)
{
    cv::Mat hsv, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_green(40, 50, 50);
    cv::Scalar upper_green(80, 255, 255);
    cv::inRange(hsv, lower_green, upper_green, mask);
    return mask;
}

cv::Mat TurtlebotProject::detectBlueLine(const cv::Mat &image)
{
    cv::Mat hsv, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_blue(100, 50, 50);
    cv::Scalar upper_blue(130, 255, 255);
    cv::inRange(hsv, lower_blue, upper_blue, mask);
    return mask;
}
cv::Mat TurtlebotProject::detectRedLine(const cv::Mat &image)
{
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // HSV 색공간에서 빨간색은 색상(H) 값이 0-10과 160-180 범위에 있으므로
    // 두 개의 마스크를 만들어 합칩니다
    cv::Scalar lower_red1(0, 100, 100);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100);
    cv::Scalar upper_red2(180, 255, 255);

    cv::inRange(hsv, lower_red1, upper_red1, mask1);
    cv::inRange(hsv, lower_red2, upper_red2, mask2);
    mask = mask1 | mask2;

    return mask;
}

void TurtlebotProject::imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        int height = frame.rows;
        int width = frame.cols;

        // ROI 영역 정의
        cv::Point2f src_vertices[4];
        cv::Point2f dst_vertices[4];

        if (current_direction == "Right`")
        {
            src_vertices[0] = cv::Point2f(width * 0.55f, height * 0.95f);
            src_vertices[1] = cv::Point2f(width * 0.65f, height * 0.95f);
            src_vertices[2] = cv::Point2f(width * 0.8f, height * 1.0f);
            src_vertices[3] = cv::Point2f(width * 0.55f, height * 1.0f);
        }
        else if (current_direction == "Left`")
        {
            src_vertices[0] = cv::Point2f(width * 0.35f, height * 0.95f);
            src_vertices[1] = cv::Point2f(width * 0.55f, height * 0.95f);
            src_vertices[2] = cv::Point2f(width * 0.55f, height * 1.0f);
            src_vertices[3] = cv::Point2f(width * 0.2f, height * 1.0f);
        }
        else
        {
            src_vertices[0] = cv::Point2f(width * 0.3f, height * 0.85f);
            src_vertices[1] = cv::Point2f(width * 0.7f, height * 0.85f);
            src_vertices[2] = cv::Point2f(width * 0.8f, height * 1.0f);
            src_vertices[3] = cv::Point2f(width * 0.2f, height * 1.0f);
        }

        dst_vertices[0] = cv::Point2f(width * 0.35f, height * 0.6f);
        dst_vertices[1] = cv::Point2f(width * 0.65f, height * 0.6f);
        dst_vertices[2] = cv::Point2f(width * 0.65f, height);
        dst_vertices[3] = cv::Point2f(width * 0.35f, height);

        cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);
        cv::Mat birds_eye_view;
        cv::warpPerspective(frame, birds_eye_view, perspective_matrix, frame.size());

        cv::Mat yellow_mask = detectYellowLine(birds_eye_view);
        cv::Mat white_mask = detectWhiteLine(birds_eye_view);
        cv::Mat green_mask = detectGreenLine(birds_eye_view);
        cv::Mat blue_mask = detectBlueLine(birds_eye_view);
        cv::Mat red_mask = detectRedLine(birds_eye_view);

        int yellow_pixels = cv::countNonZero(yellow_mask);
        int white_pixels = cv::countNonZero(white_mask);
        int green_pixels = cv::countNonZero(green_mask);
        int blue_pixels = cv::countNonZero(blue_mask);
        int red_pixels = cv::countNonZero(red_mask);

        if (red_pixels > 100)
        {
            should_stop = true;
            RCLCPP_INFO(this->get_logger(), "Red line detected! Stopping the robot.");
        }

        if (blue_pixels > 100)
        {
            if (!is_blue_line_detected)
            {
                blue_line_count++;
                is_blue_line_detected = true;
                RCLCPP_INFO(this->get_logger(), "Blue line detected! Count: %d", blue_line_count);
            }
        }
        else
        {
            is_blue_line_detected = false;
        }

        // 교차로 도달 확인
        if (blue_line_count == 2)
        {
            intersection = true;
        }

        // 명령 생성
        geometry_msgs::msg::Twist cmd_vel_msg;
        std::string direction;
        std::string speed_status;

        // 가속/감속 처리
        if (green_pixels > 100)
        {
            is_accelerated = true;
        }
        else if (blue_pixels > 100)
        {
            is_accelerated = false;
        }

        cmd_vel_msg.linear.x = is_accelerated ? linear_speed * 1.5 : linear_speed;
        speed_status = is_accelerated ? "SPEED UP!" : "NORMAL";

        // 방향 제어
        if (should_stop)
        {
            // 빨간색 선이 감지되면 로봇을 정지
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            direction = "STOP";
            speed_status = "STOPPED";
        }
        else
        {
            // 기존의 방향 제어 로직
            if (intersection)
            {
                if (current_direction == "Left")
                {
                    cmd_vel_msg.linear.x = linear_speed * 0.8;
                    cmd_vel_msg.angular.z = angular_speed;
                    direction = "LEFT TURN AT INTERSECTION";
                }
                else if (current_direction == "Right")
                {
                    cmd_vel_msg.linear.x = linear_speed * 0.8;
                    cmd_vel_msg.angular.z = -angular_speed;
                    direction = "RIGHT TURN AT INTERSECTION";
                }
            }
            else
            {
                if (yellow_pixels > white_pixels)
                {
                    cmd_vel_msg.angular.z = angular_speed;
                    direction = "LEFT";
                }
                else if (white_pixels > yellow_pixels)
                {
                    cmd_vel_msg.angular.z = -angular_speed;
                    direction = "RIGHT";
                }
                else
                {
                    cmd_vel_msg.angular.z = 0.0;
                    direction = "STRAIGHT";
                }
                cmd_vel_msg.linear.x = is_accelerated ? linear_speed * 1.5 : linear_speed;
                speed_status = is_accelerated ? "SPEED UP!" : "NORMAL";
            }
        }

        delayed_commands.push(cmd_vel_msg);

        if (delayed_commands.size() > delay_frames)
        {
            cmd_vel_publisher->publish(delayed_commands.front());
            delayed_commands.pop();
        }

        // 화면 표시
        cv::Mat display_frame = frame.clone();
        for (int i = 0; i < 4; i++)
        {
            cv::line(display_frame, src_vertices[i], src_vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2);
        }

        cv::Mat birds_eye_display = birds_eye_view.clone();
        birds_eye_display.setTo(cv::Scalar(0, 255, 255), yellow_mask);
        birds_eye_display.setTo(cv::Scalar(255, 255, 255), white_mask);
        birds_eye_display.setTo(cv::Scalar(0, 255, 0), green_mask);
        birds_eye_display.setTo(cv::Scalar(255, 0, 0), blue_mask);
        birds_eye_display.setTo(cv::Scalar(0, 0, 255), red_mask);
        cv::putText(birds_eye_display,
                    "Yellow: " + std::to_string(yellow_pixels),
                    cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);

        cv::putText(birds_eye_display,
                    "White: " + std::to_string(white_pixels),
                    cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 255), 2);

        cv::putText(birds_eye_display,
                    "Mode: " + current_direction,
                    cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 255), 2);

        cv::putText(birds_eye_display,
                    "Speed: " + speed_status + " (Queue: " +
                        std::to_string(delayed_commands.size()) + "/" +
                        std::to_string(delay_frames) + ")",
                    cv::Point(10, 120),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 255), 2);

        cv::putText(birds_eye_display,
                    "Next Direction: " + direction,
                    cv::Point(10, 150),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 255), 2);

        cv::putText(birds_eye_display,
                    "Blue Line Count: " + std::to_string(blue_line_count),
                    cv::Point(10, 180),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 255), 2);

        cv::putText(birds_eye_display,
                    "Red: " + std::to_string(red_pixels),
                    cv::Point(10, 210),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 255), 2);
                    
        cv::imshow("Original View", display_frame);
        cv::imshow("Bird's Eye View", birds_eye_display);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}