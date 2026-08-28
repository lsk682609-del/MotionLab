#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "motionlab_control_cpp/astar_core.hpp"

class AStarPlanner : public rclcpp::Node
{
public:
  AStarPlanner()
  : Node("astar_planner")
  {
    auto qos = rclcpp::QoS(1)
      .reliable()
      .transient_local();

    inflation_radius_ =
      declare_parameter<double>(
      "inflation_radius",
       0.25);

    path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(
        "/planned_path", qos);

    map_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/grid_map", qos);

    map_subscription_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        qos,
        std::bind(
          &AStarPlanner::map_callback,
          this,
          std::placeholders::_1));

    goal_subscription_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(
          &AStarPlanner::goal_callback,
          this,
          std::placeholders::_1));

    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(
          &AStarPlanner::odom_callback,
          this,
          std::placeholders::_1));


    RCLCPP_INFO(
      get_logger(),
      "A* planner ready. Waiting for RViz goal on /goal_pose");
  }

private:
  void map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (
      message->info.width == 0 ||
      message->info.height == 0 ||
      message->info.resolution <= 0.0)
    {
      RCLCPP_ERROR(
      get_logger(),
      "Received invalid OccupancyGrid");
      return;
    }

    const std::size_t expected_size =
      static_cast<std::size_t>(
      message->info.width) *
      static_cast<std::size_t>(
      message->info.height);

    if (message->data.size() != expected_size) {
      RCLCPP_ERROR(
      get_logger(),
      "OccupancyGrid data size mismatch");
      return;
    }

    grid_width_ =
      static_cast<int>(message->info.width);

    grid_height_ =
      static_cast<int>(message->info.height);

    grid_resolution_ =
      message->info.resolution;

    grid_origin_x_ =
      message->info.origin.position.x;

    grid_origin_y_ =
      message->info.origin.position.y;

    std::vector<bool> raw_occupied(
      message->data.size(),
      false);

    std::size_t raw_occupied_count = 0;

    for (std::size_t i = 0;
      i < message->data.size();
      ++i)
    {
      const int value =
        static_cast<int>(
        message->data[i]);

      raw_occupied[i] =
        value < 0 || value >= 50;

      if (raw_occupied[i]) {
        ++raw_occupied_count;
      }
    }

    inflate_obstacles(raw_occupied);

    std::size_t inflated_occupied_count = 0;

    for (const bool occupied : occupied_) {
      if (occupied) {
        ++inflated_occupied_count;
      }
    }

    const bool first_map = !map_received_;
    map_received_ = true;

    if (first_map) {
      RCLCPP_INFO(
  get_logger(),
  "Map received: %d x %d, resolution=%.3f, "
  "origin=(%.3f, %.3f), "
  "inflation=%.2f m, raw=%zu, inflated=%zu",
  grid_width_,
  grid_height_,
  grid_resolution_,
  grid_origin_x_,
  grid_origin_y_,
  inflation_radius_,
  raw_occupied_count,
  inflated_occupied_count);
    }

    publish_grid_map();

    if (pending_goal_ && pose_received_) {
      pending_goal_ = false;
      plan_to_goal(goal_x_, goal_y_);
    }
  }

  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    robot_x_ = message->pose.pose.position.x;
    robot_y_ = message->pose.pose.position.y;

    const bool first_pose = !pose_received_;
    pose_received_ = true;

    if (first_pose) {
      RCLCPP_INFO(
        get_logger(),
        "Robot pose received: (%.2f, %.2f)",
        robot_x_,
        robot_y_);
    }

    if (pending_goal_ && map_received_) {
      pending_goal_ = false;
      plan_to_goal(goal_x_, goal_y_);
    }
  }

  void goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    goal_x_ = message->pose.position.x;
    goal_y_ = message->pose.position.y;

    RCLCPP_INFO(
      get_logger(),
      "New navigation goal received: (%.2f, %.2f)",
      goal_x_,
      goal_y_);

    if (!pose_received_ || !map_received_) {
      pending_goal_ = true;

      RCLCPP_WARN(
    get_logger(),
    "Goal stored: pose_received=%s, map_received=%s",
    pose_received_ ? "true" : "false",
    map_received_ ? "true" : "false");

      return;
    }

    plan_to_goal(goal_x_, goal_y_);
  }

  void plan_to_goal(
    double goal_x,
    double goal_y)
  {
    const int start_grid_x =
      world_to_grid_x(robot_x_);

    const int start_grid_y =
      world_to_grid_y(robot_y_);

    const int goal_grid_x =
      world_to_grid_x(goal_x);

    const int goal_grid_y =
      world_to_grid_y(goal_y);

    if (!inside_grid(start_grid_x, start_grid_y)) {
      RCLCPP_ERROR(
        get_logger(),
        "Robot start position is outside the planning map");

      publish_empty_path();
      return;
    }

    if (!inside_grid(goal_grid_x, goal_grid_y)) {
      RCLCPP_WARN(
        get_logger(),
        "Goal (%.2f, %.2f) is outside the planning map",
        goal_x,
        goal_y);

      publish_empty_path();
      return;
    }

    if (occupied_[to_index(start_grid_x, start_grid_y)]) {
      RCLCPP_ERROR(
        get_logger(),
        "Robot start position lies inside an obstacle");

      publish_empty_path();
      return;
    }

    if (occupied_[to_index(goal_grid_x, goal_grid_y)]) {
      RCLCPP_WARN(
        get_logger(),
        "Goal lies inside an obstacle");

      publish_empty_path();
      return;
    }

    AStarCore astar_core(
      grid_width_,
      grid_height_,
      grid_resolution_,
      occupied_);

    const auto path_indices =
      astar_core.plan(
        start_grid_x,
        start_grid_y,
        goal_grid_x,
        goal_grid_y);

    if (path_indices.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "A* failed: no path from (%.2f, %.2f) to (%.2f, %.2f)",
        robot_x_,
        robot_y_,
        goal_x,
        goal_y);

      publish_empty_path();
      return;
    }

    publish_path(path_indices);

    RCLCPP_INFO(
      get_logger(),
      "Planning succeeded: start=(%.2f, %.2f), goal=(%.2f, %.2f)",
      robot_x_,
      robot_y_,
      goal_x,
      goal_y);
  }

  int to_index(int x, int y) const
  {
    return y * grid_width_ + x;
  }

  std::pair<int, int> from_index(
    int index) const
  {
    return {
      index % grid_width_,
      index / grid_width_
    };
  }

  bool inside_grid(int x, int y) const
  {
    return
      x >= 0 &&
      x < grid_width_ &&
      y >= 0 &&
      y < grid_height_;
  }

  void inflate_obstacles(
    const std::vector<bool> & raw_occupied)
  {
    occupied_ = raw_occupied;

    if (
      inflation_radius_ <= 0.0 ||
      grid_resolution_ <= 0.0)
    {
      return;
    }

    const int inflation_cells =
      static_cast<int>(
      std::ceil(
      inflation_radius_ /
      grid_resolution_));

    const double radius_squared =
      inflation_radius_ *
      inflation_radius_;

    for (int y = 0; y < grid_height_; ++y) {
      for (int x = 0; x < grid_width_; ++x) {

        if (!raw_occupied[to_index(x, y)]) {
          continue;
        }

        for (int dy = -inflation_cells;
          dy <= inflation_cells;
          ++dy)
        {
          for (int dx = -inflation_cells;
            dx <= inflation_cells;
            ++dx)
          {
            const int nx = x + dx;
            const int ny = y + dy;

            if (!inside_grid(nx, ny)) {
              continue;
            }

            const double distance_x =
              static_cast<double>(dx) *
              grid_resolution_;

            const double distance_y =
              static_cast<double>(dy) *
              grid_resolution_;

            const double distance_squared =
              distance_x * distance_x +
              distance_y * distance_y;

            if (distance_squared <= radius_squared) {
              occupied_[to_index(nx, ny)] = true;
            }
          }
        }
      }
    }
  }

  int world_to_grid_x(double x) const
  {
    return static_cast<int>(
      std::floor(
        (x - grid_origin_x_) /
      grid_resolution_));
  }

  int world_to_grid_y(double y) const
  {
    return static_cast<int>(
      std::floor(
        (y - grid_origin_y_) /
      grid_resolution_));
  }

  double grid_to_world_x(int grid_x) const
  {
    return
      grid_origin_x_ +
      (static_cast<double>(grid_x) + 0.5) *
      grid_resolution_;
  }

  double grid_to_world_y(int grid_y) const
  {
    return
      grid_origin_y_ +
      (static_cast<double>(grid_y) + 0.5) *
      grid_resolution_;
  }

  void build_obstacle_map()
  {
    occupied_.assign(
      grid_width_ * grid_height_,
      false);

    const int wall_x = 13;

    for (int y = 2; y <= 18; ++y) {
      if (y == 14 || y == 15) {
        continue;
      }

      occupied_[to_index(wall_x, y)] = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "Obstacle map created");
  }

  void publish_grid_map()
  {
    nav_msgs::msg::OccupancyGrid map;

    map.header.stamp = now();
    map.header.frame_id = "map";

    map.info.resolution =
      grid_resolution_;

    map.info.width =
      grid_width_;

    map.info.height =
      grid_height_;

    map.info.origin.position.x =
      grid_origin_x_;

    map.info.origin.position.y =
      grid_origin_y_;

    map.info.origin.orientation.w = 1.0;

    map.data.resize(occupied_.size());

    for (std::size_t i = 0;
      i < occupied_.size();
      ++i)
    {
      map.data[i] =
        occupied_[i] ? 100 : 0;
    }

    map_publisher_->publish(map);

    RCLCPP_INFO(
      get_logger(),
      "Published occupancy grid on /grid_map");
  }

  bool line_is_free(
    int start_index,
    int end_index) const
  {
    auto [x0, y0] =
      from_index(start_index);

    const auto [x1, y1] =
      from_index(end_index);

    const int dx =
      std::abs(x1 - x0);

    const int sx =
      x0 < x1 ? 1 : -1;

    const int dy =
      -std::abs(y1 - y0);

    const int sy =
      y0 < y1 ? 1 : -1;

    int error = dx + dy;

    while (true) {
      if (
        !inside_grid(x0, y0) ||
        occupied_[to_index(x0, y0)])
      {
        return false;
      }

      if (x0 == x1 && y0 == y1) {
        break;
      }

      const int previous_x = x0;
      const int previous_y = y0;

      const int doubled_error =
        2 * error;

      if (doubled_error >= dy) {
        error += dy;
        x0 += sx;
      }

      if (doubled_error <= dx) {
        error += dx;
        y0 += sy;
      }

      const bool moved_diagonally =
        x0 != previous_x &&
        y0 != previous_y;

      if (moved_diagonally) {
        const int side_a_x = x0;
        const int side_a_y = previous_y;

        const int side_b_x = previous_x;
        const int side_b_y = y0;

        if (
          !inside_grid(side_a_x, side_a_y) ||
          !inside_grid(side_b_x, side_b_y) ||
          occupied_[to_index(side_a_x, side_a_y)] ||
          occupied_[to_index(side_b_x, side_b_y)])
        {
          return false;
        }
      }
    }

    return true;
  }

  std::vector<int> simplify_path(
    const std::vector<int> & raw_path) const
  {
    if (raw_path.size() <= 2) {
      return raw_path;
    }

    std::vector<int> simplified;

    std::size_t current = 0;

    simplified.push_back(
      raw_path.front());

    while (current < raw_path.size() - 1) {
      std::size_t next =
        raw_path.size() - 1;

      while (
        next > current + 1 &&
        !line_is_free(
          raw_path[current],
          raw_path[next]))
      {
        --next;
      }

      simplified.push_back(
        raw_path[next]);

      current = next;
    }

    return simplified;
  }

  std::vector<std::pair<double, double>>
  densify_path(
    const std::vector<int> & sparse_path) const
  {
    std::vector<std::pair<double, double>>
    dense_path;

    if (sparse_path.empty()) {
      return dense_path;
    }

    const double spacing = 0.1;

    for (std::size_t segment = 0;
      segment + 1 < sparse_path.size();
      ++segment)
    {
      const auto [x0_grid, y0_grid] =
        from_index(sparse_path[segment]);

      const auto [x1_grid, y1_grid] =
        from_index(sparse_path[segment + 1]);

      const double x0 =
        grid_to_world_x(x0_grid);

      const double y0 =
        grid_to_world_y(y0_grid);

      const double x1 =
        grid_to_world_x(x1_grid);

      const double y1 =
        grid_to_world_y(y1_grid);

      const double distance =
        std::hypot(
          x1 - x0,
          y1 - y0);

      const int steps =
        std::max(
          1,
          static_cast<int>(
          std::ceil(
              distance / spacing)));

      for (int step = 0;
        step < steps;
        ++step)
      {
        const double ratio =
          static_cast<double>(step) /
          static_cast<double>(steps);

        dense_path.push_back({
          x0 + ratio * (x1 - x0),
          y0 + ratio * (y1 - y0)
        });
      }
    }

    const auto [last_x_grid, last_y_grid] =
      from_index(sparse_path.back());

    dense_path.push_back({
      grid_to_world_x(last_x_grid),
      grid_to_world_y(last_y_grid)
    });

    return dense_path;
  }

  void publish_path(
    const std::vector<int> & path_indices)
  {
    nav_msgs::msg::Path path;

    path.header.stamp = now();
    path.header.frame_id = "map";

    const auto simplified_path =
      simplify_path(path_indices);

    const auto dense_path =
      densify_path(simplified_path);

    for (const auto & point : dense_path) {
      geometry_msgs::msg::PoseStamped pose;

      pose.header = path.header;
      pose.pose.position.x = point.first;
      pose.pose.position.y = point.second;
      pose.pose.orientation.w = 1.0;

      path.poses.push_back(pose);
    }

    path_publisher_->publish(path);

    RCLCPP_INFO(
      get_logger(),
      "Path processing: raw=%zu, simplified=%zu, dense=%zu",
      path_indices.size(),
      simplified_path.size(),
      dense_path.size());

    RCLCPP_INFO(
      get_logger(),
      "Published replanned path on /planned_path");
  }

  void publish_empty_path()
  {
    nav_msgs::msg::Path path;

    path.header.stamp = now();
    path.header.frame_id = "map";

    path_publisher_->publish(path);
  }

  rclcpp::Publisher<
    nav_msgs::msg::Path>::SharedPtr
    path_publisher_;

  rclcpp::Publisher<
    nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_publisher_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_subscription_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_subscription_;

  std::vector<bool> occupied_;

  int grid_width_{0};
  int grid_height_{0};

  double grid_resolution_{0.0};

  double grid_origin_x_{0.0};
  double grid_origin_y_{0.0};

  double inflation_radius_{0.25};

  double robot_x_{0.0};
  double robot_y_{0.0};

  double goal_x_{0.0};
  double goal_y_{0.0};

  bool pose_received_{false};
  bool map_received_{false};
  bool pending_goal_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<AStarPlanner>());

  rclcpp::shutdown();

  return 0;
}
