#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class WaypointManager : public rclcpp::Node
{
public:
  WaypointManager()
  : Node("waypoint_manager"),
    waypoints_{
      {2.0, 8.0},
      {2.0, 2.0},
      {8.0, 2.0},
      {8.0, 8.0}}
  {
    goal_publisher_ =
      create_publisher<geometry_msgs::msg::Point>(
        "/goal_point", 10);

    pose_subscription_ =
      create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose",
        10,
        std::bind(
          &WaypointManager::pose_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Waypoint manager started with %zu waypoints",
      waypoints_.size());
  }

private:
  void publish_current_goal()
  {
    const auto & waypoint = waypoints_[current_index_];

    geometry_msgs::msg::Point message;
    message.x = waypoint.first;
    message.y = waypoint.second;
    message.z = 0.0;

    goal_publisher_->publish(message);

    RCLCPP_INFO(
      get_logger(),
      "Publishing waypoint %zu: (%.2f, %.2f)",
      current_index_ + 1,
      message.x,
      message.y);
  }

  void pose_callback(
    const turtlesim::msg::Pose::SharedPtr pose)
  {
    if (finished_) {
      return;
    }

    if (!goal_sent_) {
      publish_current_goal();
      goal_sent_ = true;
      return;
    }

    const auto & waypoint = waypoints_[current_index_];
    const double dx = waypoint.first - pose->x;
    const double dy = waypoint.second - pose->y;
    const double distance = std::hypot(dx, dy);

    if (distance < waypoint_tolerance_) {
      RCLCPP_INFO(
        get_logger(),
        "Waypoint %zu reached",
        current_index_ + 1);

      ++current_index_;

      if (current_index_ >= waypoints_.size()) {
        finished_ = true;
        RCLCPP_INFO(get_logger(), "All waypoints completed");
        return;
      }

      publish_current_goal();
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
    goal_publisher_;

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr
    pose_subscription_;

  std::vector<std::pair<double, double>> waypoints_;
  std::size_t current_index_{0};

  bool goal_sent_{false};
  bool finished_{false};

  double waypoint_tolerance_{0.15};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}
