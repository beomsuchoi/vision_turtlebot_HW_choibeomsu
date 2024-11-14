#include "rclcpp/rclcpp.hpp"
#include "maze_lidar_sensor/maze_lidar_sensor.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeLidarSensor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}