#include <cmath>
#include <memory>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher()
  : Node("path_publisher")
  {
    auto qos = rclcpp::QoS(1)
      .reliable()
      .transient_local();

    publisher_ =
      create_publisher<nav_msgs::msg::Path>(
        "/planned_path", qos);

    publish_path();
  }

private:
  void publish_path()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "turtlesim";

    for (int i = 0; i <= 100; ++i) {
      const double x = 5.5 + 0.045 * i;
      const double y =
        5.5 + 1.5 * std::sin(1.2 * (x - 5.5));

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path.poses.push_back(pose);
    }

    publisher_->publish(path);

    RCLCPP_INFO(
      get_logger(),
      "Published path with %zu points",
      path.poses.size());
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
    publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}
