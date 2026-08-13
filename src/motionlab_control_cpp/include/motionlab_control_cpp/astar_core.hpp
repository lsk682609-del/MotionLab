#ifndef MOTIONLAB_CONTROL_CPP__ASTAR_CORE_HPP_
#define MOTIONLAB_CONTROL_CPP__ASTAR_CORE_HPP_

#include <vector>

class AStarCore
{
public:
  AStarCore(
    int width,
    int height,
    double resolution,
    const std::vector<bool> & occupied);

  std::vector<int> plan(
    int start_x,
    int start_y,
    int goal_x,
    int goal_y) const;

private:
  int to_index(int x, int y) const;
  bool inside_grid(int x, int y) const;

  double heuristic(
    int x,
    int y,
    int goal_x,
    int goal_y) const;

  int width_;
  int height_;
  double resolution_;
  std::vector<bool> occupied_;
};

#endif  // MOTIONLAB_CONTROL_CPP__ASTAR_CORE_HPP_
