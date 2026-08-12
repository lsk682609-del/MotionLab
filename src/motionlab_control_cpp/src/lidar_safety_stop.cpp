#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSafetyStop : public rclcpp::Node
{
public:
  LidarSafetyStop() : Node("lidar_safety_stop")
  {
    stop_distance_ = declare_parameter<double>("stop_distance", 0.6);
    front_angle_ = declare_parameter<double>("front_angle_deg", 30.0) * M_PI / 180.0;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::Twist safe_cmd = *msg;

        if (obstacle_detected_ && safe_cmd.linear.x > 0.0) {
          safe_cmd.linear.x = 0.0;
          safe_cmd.angular.z = 0.0;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Emergency stop: obstacle at %.2f m", min_front_range_);
        }

        cmd_pub_->publish(safe_cmd);
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        min_front_range_ = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
          const double angle = scan->angle_min + i * scan->angle_increment;
          const double range = scan->ranges[i];

          if (std::abs(angle) <= front_angle_ &&
              std::isfinite(range) &&
              range >= scan->range_min &&
              range <= scan->range_max) {
            min_front_range_ = std::min(min_front_range_, range);
          }
        }

        obstacle_detected_ = min_front_range_ < stop_distance_;
      });

    RCLCPP_INFO(
      get_logger(),
      "Safety stop ready: stop_distance=%.2f m, front_angle=%.1f deg",
      stop_distance_, front_angle_ * 180.0 / M_PI);
  }

private:
  double stop_distance_;
  double front_angle_;
  double min_front_range_{std::numeric_limits<double>::infinity()};
  bool obstacle_detected_{true};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSafetyStop>());
  rclcpp::shutdown();
  return 0;
}
