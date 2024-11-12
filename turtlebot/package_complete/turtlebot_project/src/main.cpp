#include "rclcpp/rclcpp.hpp"
#include "turtlebot_project/turtlebot_project.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlebotProject>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}