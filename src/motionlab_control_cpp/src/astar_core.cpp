#include "motionlab_control_cpp/astar_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace
{

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

}  // namespace

AStarCore::AStarCore(
  int width,
  int height,
  double resolution,
  const std::vector<bool> & occupied)
: width_(width),
  height_(height),
  resolution_(resolution),
  occupied_(occupied)
{
}

int AStarCore::to_index(int x, int y) const
{
  return y * width_ + x;
}

bool AStarCore::inside_grid(int x, int y) const
{
  return x >= 0 && x < width_ &&
         y >= 0 && y < height_;
}

double AStarCore::heuristic(
  int x,
  int y,
  int goal_x,
  int goal_y) const
{
  return std::hypot(
    static_cast<double>(goal_x - x),
    static_cast<double>(goal_y - y)) *
         resolution_;
}

std::vector<int> AStarCore::plan(
  int start_x,
  int start_y,
  int goal_x,
  int goal_y) const
{
  if (
    !inside_grid(start_x, start_y) ||
    !inside_grid(goal_x, goal_y))
  {
    return {};
  }

  const int start_index = to_index(start_x, start_y);
  const int goal_index = to_index(goal_x, goal_y);

  if (
    occupied_[start_index] ||
    occupied_[goal_index])
  {
    return {};
  }

  const int cell_count = width_ * height_;

  std::vector<double> g_cost(
    cell_count,
    std::numeric_limits<double>::infinity());

  std::vector<int> parent(cell_count, -1);
  std::vector<bool> closed(cell_count, false);

  std::priority_queue<
    SearchState,
    std::vector<SearchState>,
    CompareState> open_set;

  g_cost[start_index] = 0.0;

  open_set.push({
    start_index,
    heuristic(start_x, start_y, goal_x, goal_y)
  });

  const int direction_x[8] =
  {1, 1, 0, -1, -1, -1, 0, 1};

  const int direction_y[8] =
  {0, 1, 1, 1, 0, -1, -1, -1};

  bool found = false;

  while (!open_set.empty()) {
    const SearchState current = open_set.top();
    open_set.pop();

    if (closed[current.index]) {
      continue;
    }

    closed[current.index] = true;

    if (current.index == goal_index) {
      found = true;
      break;
    }

    const int current_x = current.index % width_;
    const int current_y = current.index / width_;

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
        resolution_;

      const double tentative_cost =
        g_cost[current.index] + step_cost;

      if (tentative_cost < g_cost[next_index]) {
        g_cost[next_index] = tentative_cost;
        parent[next_index] = current.index;

        const double f_cost =
          tentative_cost +
          heuristic(
            next_x,
            next_y,
            goal_x,
            goal_y);

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

    current_index = parent[current_index];
  }

  std::reverse(path.begin(), path.end());

  return path;
}
