#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSafetyStop : public rclcpp::Node
{
public:
  LidarSafetyStop()
  : Node("lidar_safety_stop")
  {
    stop_distance_ = declare_parameter<double>("stop_distance", 0.35);
    release_margin_ = declare_parameter<double>("release_margin", 0.10);
    front_angle_ = declare_parameter<double>("front_angle_deg", 30.0) * M_PI / 180.0;
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.5);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);

    last_scan_time_ = this->now();
    last_cmd_time_ = this->now();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = this->now();
        cmd_timed_out_ = false;
        geometry_msgs::msg::Twist safe_cmd = *msg;


        if (obstacle_detected_ && safe_cmd.linear.x > 0.0) {
          safe_cmd.linear.x = 0.0;

          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Forward motion blocked: obstacle at %.2f m",
            min_front_range_);
        }

        cmd_pub_->publish(safe_cmd);
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        last_scan_time_ = this->now();
        scan_timed_out_ = false;
        min_front_range_ = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
          const double angle = scan->angle_min + i * scan->angle_increment;
          const double range = scan->ranges[i];

          if (std::abs(angle) <= front_angle_ &&
          std::isfinite(range) &&
          range >= scan->range_min &&
          range <= scan->range_max)
          {
            min_front_range_ = std::min(min_front_range_, range);
          }
        }

        if (!obstacle_detected_ && min_front_range_ < stop_distance_) {
          obstacle_detected_ = true;
          RCLCPP_WARN(get_logger(), "Safety state: STOP at %.2f m", min_front_range_);
        } else if (obstacle_detected_ && min_front_range_ > stop_distance_ + release_margin_) {
          obstacle_detected_ = false;
          RCLCPP_INFO(get_logger(), "Safety state: CLEAR at %.2f m", min_front_range_);
        }
      });


    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        const auto now = this->now();

        const double scan_dt =
        (now - last_scan_time_).seconds();

        if (scan_dt > scan_timeout_) {
          if (!scan_timed_out_) {
            RCLCPP_ERROR(
              get_logger(),
              "LiDAR timeout: no /scan for %.2f s",
              scan_dt);
          }

          scan_timed_out_ = true;
          obstacle_detected_ = true;
        }

        const double cmd_dt =
        (now - last_cmd_time_).seconds();

        if (cmd_dt > cmd_timeout_) {
          if (!cmd_timed_out_) {
            geometry_msgs::msg::Twist stop_cmd;
            cmd_pub_->publish(stop_cmd);

            RCLCPP_WARN(
              get_logger(),
              "Command timeout: no /cmd_vel_raw for %.2f s, robot stopped",
              cmd_dt);
          }

          cmd_timed_out_ = true;
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "Safety stop ready: stop_distance=%.2f m, front_angle=%.1f deg",
      stop_distance_, front_angle_ * 180.0 / M_PI);
  }

private:
  double stop_distance_;
  double release_margin_;
  double front_angle_;
  double scan_timeout_;
  double cmd_timeout_;

  double min_front_range_{
    std::numeric_limits<double>::infinity()};

  bool obstacle_detected_{true};
  bool scan_timed_out_{false};
  bool cmd_timed_out_{false};

  rclcpp::Time last_scan_time_;
  rclcpp::Time last_cmd_time_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

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
