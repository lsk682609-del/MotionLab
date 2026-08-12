#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class PurePursuitController : public rclcpp::Node
{
public:
  PurePursuitController()
  : Node("pure_pursuit_controller")
  {
    lookahead_distance_ =
      declare_parameter<double>("lookahead_distance", 0.6);

    nominal_speed_ =
      declare_parameter<double>("nominal_speed", 1.0);

    max_angular_speed_ =
      declare_parameter<double>("max_angular_speed", 3.0);

    goal_tolerance_ =
      declare_parameter<double>("goal_tolerance", 0.12);

    velocity_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_raw", 10);

    auto path_qos = rclcpp::QoS(1)
      .reliable()
      .transient_local();

    path_subscription_ =
      create_subscription<nav_msgs::msg::Path>(
        "/planned_path",
        path_qos,
        std::bind(
          &PurePursuitController::path_callback,
          this,
          std::placeholders::_1));

    pose_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(
          &PurePursuitController::pose_callback,
          this,
          std::placeholders::_1));

    timer_ = create_wall_timer(
      50ms,
      std::bind(
        &PurePursuitController::control_loop,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Pure Pursuit controller started");
  }

private:
  static double normalize_angle(double angle)
  {
    return std::atan2(
      std::sin(angle),
      std::cos(angle));
  }

  void path_callback(
    const nav_msgs::msg::Path::SharedPtr message)
  {
    path_ = message->poses;
    nearest_index_ = 0;
    path_received_ = !path_.empty();
    goal_reached_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Received path with %zu points",
      path_.size());
  }

  void pose_callback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    pose_x_ = message->pose.pose.position.x;
    pose_y_ = message->pose.pose.position.y;

    const auto & q = message->pose.pose.orientation;
    pose_theta_ = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    pose_received_ = true;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist command;
    velocity_publisher_->publish(command);
  }

  std::size_t find_nearest_index()
  {
    double minimum_distance =
      std::numeric_limits<double>::max();

    std::size_t best_index = nearest_index_;

    for (std::size_t i = nearest_index_;
      i < path_.size(); ++i)
    {
      const double dx =
        path_[i].pose.position.x - pose_x_;

      const double dy =
        path_[i].pose.position.y - pose_y_;

      const double distance =
        std::hypot(dx, dy);

      if (distance < minimum_distance) {
        minimum_distance = distance;
        best_index = i;
      }
    }

    return best_index;
  }

  std::size_t find_lookahead_index(
    std::size_t start_index)
  {
    double accumulated_distance = 0.0;
    std::size_t target_index = start_index;

    while (
      target_index + 1 < path_.size() &&
      accumulated_distance < lookahead_distance_)
    {
      const auto & current =
        path_[target_index].pose.position;

      const auto & next =
        path_[target_index + 1].pose.position;

      accumulated_distance += std::hypot(
        next.x - current.x,
        next.y - current.y);

      ++target_index;
    }

    return target_index;
  }

  void control_loop()
  {
    if (!path_received_ || !pose_received_) {
      return;
    }

    const auto & final_point =
      path_.back().pose.position;

    const double final_distance = std::hypot(
      final_point.x - pose_x_,
      final_point.y - pose_y_);

    if (final_distance < goal_tolerance_) {
      publish_stop();

      if (!goal_reached_) {
        goal_reached_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Path completed: distance=%.3f",
          final_distance);
        const double rmse =
          error_sample_count_ > 0 ?
          std::sqrt(
      squared_error_sum_ /
      static_cast<double>(error_sample_count_)) :
          0.0;

        RCLCPP_INFO(
  get_logger(),
  "Tracking metrics: RMSE=%.3f, "
  "max_error=%.3f, samples=%zu",
  rmse,
  max_tracking_error_,
  error_sample_count_);
      }

      return;
    }

    nearest_index_ = find_nearest_index();
    const auto & nearest_point =
      path_[nearest_index_].pose.position;

    const double tracking_error = std::hypot(
  nearest_point.x - pose_x_,
  nearest_point.y - pose_y_);

    squared_error_sum_ +=
      tracking_error * tracking_error;

    max_tracking_error_ = std::max(
  max_tracking_error_,
  tracking_error);

    ++error_sample_count_;
    const std::size_t target_index =
      find_lookahead_index(nearest_index_);

    const auto & target =
      path_[target_index].pose.position;

    const double dx = target.x - pose_x_;
    const double dy = target.y - pose_y_;
    const double target_distance =
      std::max(0.05, std::hypot(dx, dy));

    const double target_heading =
      std::atan2(dy, dx);

    const double heading_error =
      normalize_angle(
        target_heading - pose_theta_);

    const double curvature =
      2.0 * std::sin(heading_error) /
      target_distance;

    double linear_speed =
      nominal_speed_ /
      (1.0 + std::abs(curvature));

    double angular_speed =
      linear_speed * curvature;

    if (std::abs(heading_error) > 1.0) {
      linear_speed = 0.0;
      angular_speed = 2.0 * heading_error;
    }

    linear_speed = std::min(
      linear_speed,
      std::max(0.15, final_distance));

    angular_speed = std::clamp(
      angular_speed,
      -max_angular_speed_,
      max_angular_speed_);

    geometry_msgs::msg::Twist command;
    command.linear.x = linear_speed;
    command.angular.z = angular_speed;
    velocity_publisher_->publish(command);

    ++control_count_;

    if (control_count_ % 20 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "nearest=%zu, target=%zu, error=%.2f, "
        "curvature=%.2f, v=%.2f, omega=%.2f",
        nearest_index_,
        target_index,
        heading_error,
        curvature,
        linear_speed,
        angular_speed);
    }
  }

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr
    velocity_publisher_;

  rclcpp::Subscription<
    nav_msgs::msg::Path>::SharedPtr
    path_subscription_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    pose_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<
    geometry_msgs::msg::PoseStamped> path_;

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_theta_{0.0};

  std::size_t nearest_index_{0};
  std::size_t control_count_{0};
  std::size_t error_sample_count_{0};
  double squared_error_sum_{0.0};
  double max_tracking_error_{0.0};

  bool path_received_{false};
  bool pose_received_{false};
  bool goal_reached_{false};

  double lookahead_distance_;
  double nominal_speed_;
  double max_angular_speed_;
  double goal_tolerance_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<PurePursuitController>());

  rclcpp::shutdown();
  return 0;
}
