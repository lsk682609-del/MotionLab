#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

struct SearchState
{
  int index;
  double f_cost;
};

struct CompareState
{
  bool operator()(
    const SearchState & left,
    const SearchState & right) const
  {
    return left.f_cost > right.f_cost;
  }
};

class AStarPlanner : public rclcpp::Node
{
public:
  AStarPlanner()
  : Node("astar_planner")
  {
    start_x_ =
      declare_parameter<double>("start_x", 5.5);
    start_y_ =
      declare_parameter<double>("start_y", 5.5);
    goal_x_ =
      declare_parameter<double>("goal_x", 9.5);
    goal_y_ =
      declare_parameter<double>("goal_y", 5.5);

    auto qos = rclcpp::QoS(1)
      .reliable()
      .transient_local();

    path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(
        "/planned_path", qos);
map_publisher_ =
  create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/grid_map", qos);

    build_obstacle_map();
publish_grid_map();

    const auto path_indices = plan();

    if (path_indices.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "A* failed to find a path");
      return;
    }

    publish_path(path_indices);
  }

private:
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
    return x >= 0 && x < grid_width_ &&
           y >= 0 && y < grid_height_;
  }

  int world_to_grid(double value) const
  {
    return static_cast<int>(
      std::round(
        (value - grid_origin_) /
        grid_resolution_));
  }

  double grid_to_world(int value) const
  {
    return grid_origin_ +
           value * grid_resolution_;
  }

  double heuristic(
    int x,
    int y,
    int goal_x,
    int goal_y) const
  {
    return std::hypot(
      static_cast<double>(goal_x - x),
      static_cast<double>(goal_y - y)) *
      grid_resolution_;
  }

  void build_obstacle_map()
  {
    occupied_.assign(
      grid_width_ * grid_height_,
      false);

    // Vertical wall at x = 7.0.
    // The gap at y = 7.5 to 8.0 forces a detour.
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

  map.info.resolution = grid_resolution_;
  map.info.width = grid_width_;
  map.info.height = grid_height_;

  map.info.origin.position.x =
    grid_origin_ - grid_resolution_ / 2.0;

  map.info.origin.position.y =
    grid_origin_ - grid_resolution_ / 2.0;

  map.info.origin.orientation.w = 1.0;

  map.data.resize(occupied_.size());

  for (std::size_t i = 0;
    i < occupied_.size(); ++i)
  {
    map.data[i] =
      occupied_[i] ? 100 : 0;
  }

  map_publisher_->publish(map);

  RCLCPP_INFO(
    get_logger(),
    "Published occupancy grid on /grid_map");
}

  std::vector<int> plan()
  {
    const int start_grid_x =
      world_to_grid(start_x_);
    const int start_grid_y =
      world_to_grid(start_y_);
    const int goal_grid_x =
      world_to_grid(goal_x_);
    const int goal_grid_y =
      world_to_grid(goal_y_);

    if (
      !inside_grid(start_grid_x, start_grid_y) ||
      !inside_grid(goal_grid_x, goal_grid_y))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Start or goal is outside the grid");
      return {};
    }

    const int start_index =
      to_index(start_grid_x, start_grid_y);
    const int goal_index =
      to_index(goal_grid_x, goal_grid_y);

    if (
      occupied_[start_index] ||
      occupied_[goal_index])
    {
      RCLCPP_ERROR(
        get_logger(),
        "Start or goal is occupied");
      return {};
    }

    const int cell_count =
      grid_width_ * grid_height_;

    std::vector<double> g_cost(
      cell_count,
      std::numeric_limits<double>::infinity());

    std::vector<int> parent(
      cell_count,
      -1);

    std::vector<bool> closed(
      cell_count,
      false);

    std::priority_queue<
      SearchState,
      std::vector<SearchState>,
      CompareState> open_set;

    g_cost[start_index] = 0.0;

    open_set.push({
      start_index,
      heuristic(
        start_grid_x,
        start_grid_y,
        goal_grid_x,
        goal_grid_y)
    });

    const int direction_x[8] =
      {1, 1, 0, -1, -1, -1, 0, 1};

    const int direction_y[8] =
      {0, 1, 1, 1, 0, -1, -1, -1};

    int expanded_nodes = 0;
    bool found = false;

    while (!open_set.empty()) {
      const SearchState current =
        open_set.top();
      open_set.pop();

      if (closed[current.index]) {
        continue;
      }

      closed[current.index] = true;
      ++expanded_nodes;

      if (current.index == goal_index) {
        found = true;
        break;
      }

      const auto [current_x, current_y] =
        from_index(current.index);

      for (int i = 0; i < 8; ++i) {
        const int next_x =
          current_x + direction_x[i];

        const int next_y =
          current_y + direction_y[i];

        if (!inside_grid(next_x, next_y)) {
          continue;
        }

        const int next_index =
          to_index(next_x, next_y);

        if (
          occupied_[next_index] ||
          closed[next_index])
        {
          continue;
        }

        // Prevent diagonal corner cutting.
        if (
          direction_x[i] != 0 &&
          direction_y[i] != 0)
        {
          const int side_a =
            to_index(next_x, current_y);

          const int side_b =
            to_index(current_x, next_y);

          if (
            occupied_[side_a] ||
            occupied_[side_b])
          {
            continue;
          }
        }

        const double step_cost =
          std::hypot(
            static_cast<double>(direction_x[i]),
            static_cast<double>(direction_y[i])) *
          grid_resolution_;

        const double tentative_cost =
          g_cost[current.index] +
          step_cost;

        if (tentative_cost < g_cost[next_index]) {
          g_cost[next_index] =
            tentative_cost;

          parent[next_index] =
            current.index;

          const double f_cost =
            tentative_cost +
            heuristic(
              next_x,
              next_y,
              goal_grid_x,
              goal_grid_y);

          open_set.push({
            next_index,
            f_cost
          });
        }
      }
    }

    if (!found) {
      return {};
    }

    std::vector<int> path;
    int current_index = goal_index;

    while (current_index != -1) {
      path.push_back(current_index);

      if (current_index == start_index) {
        break;
      }

      current_index =
        parent[current_index];
    }

    std::reverse(
      path.begin(),
      path.end());

    RCLCPP_INFO(
      get_logger(),
      "A* success: expanded=%d, "
      "path_points=%zu, path_length=%.2f",
      expanded_nodes,
      path.size(),
      g_cost[goal_index]);

    return path;
  }
bool line_is_free(
  int start_index,
  int end_index) const
{
  auto [x0, y0] =
    from_index(start_index);

  const auto [x1, y1] =
    from_index(end_index);

  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;

  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;

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

  simplified.push_back(raw_path.front());

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
  std::vector<
    std::pair<double, double>> dense_path;

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
      grid_to_world(x0_grid);
    const double y0 =
      grid_to_world(y0_grid);

    const double x1 =
      grid_to_world(x1_grid);
    const double y1 =
      grid_to_world(y1_grid);

    const double distance =
      std::hypot(x1 - x0, y1 - y0);

    const int steps = std::max(
      1,
      static_cast<int>(
        std::ceil(distance / spacing)));

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
    grid_to_world(last_x_grid),
    grid_to_world(last_y_grid)
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
  "Path processing: raw=%zu, "
  "simplified=%zu, dense=%zu",
  path_indices.size(),
  simplified_path.size(),
  dense_path.size());
    RCLCPP_INFO(
      get_logger(),
      "Published A* path on /planned_path");
  }

  rclcpp::Publisher<
    nav_msgs::msg::Path>::SharedPtr
    path_publisher_;
rclcpp::Publisher<
  nav_msgs::msg::OccupancyGrid>::SharedPtr
  map_publisher_;

  std::vector<bool> occupied_;

  const int grid_width_{21};
  const int grid_height_{21};
  const double grid_resolution_{0.5};
  const double grid_origin_{0.5};

  double start_x_;
  double start_y_;
  double goal_x_;
  double goal_y_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}
