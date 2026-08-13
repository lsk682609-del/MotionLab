#include <algorithm>
#include <gtest/gtest.h>

#include <vector>

#include "motionlab_control_cpp/astar_core.hpp"


TEST(AStarCoreTest, FindsPathOnEmptyGrid)
{
  const int width = 5;
  const int height = 5;

  std::vector<bool> occupied(width * height, false);

  AStarCore astar(width, height, 1.0, occupied);

  const auto path = astar.plan(0, 0, 4, 4);

  ASSERT_FALSE(path.empty());

  EXPECT_EQ(path.front(), 0);
  EXPECT_EQ(path.back(), 24);
}


TEST(AStarCoreTest, FindsPathThroughWallGap)
{
  const int width = 5;
  const int height = 5;

  std::vector<bool> occupied(width * height, false);

  // Vertical wall at x = 2, with a gap at y = 2.
  occupied[0 * width + 2] = true;
  occupied[1 * width + 2] = true;
  occupied[3 * width + 2] = true;
  occupied[4 * width + 2] = true;

  AStarCore astar(width, height, 1.0, occupied);

  const auto path = astar.plan(0, 2, 4, 2);

  ASSERT_FALSE(path.empty());

  EXPECT_EQ(path.front(), 2 * width);
  EXPECT_EQ(path.back(), 2 * width + 4);

  const int gap_index = 2 * width + 2;

  EXPECT_NE(
    std::find(path.begin(), path.end(), gap_index),
    path.end());
}


TEST(AStarCoreTest, ReturnsEmptyWhenNoPathExists)
{
  const int width = 5;
  const int height = 5;

  std::vector<bool> occupied(width * height, false);

  // Solid wall completely separates start and goal.
  for (int y = 0; y < height; ++y) {
    occupied[y * width + 2] = true;
  }

  AStarCore astar(width, height, 1.0, occupied);

  const auto path = astar.plan(0, 2, 4, 2);

  EXPECT_TRUE(path.empty());
}


TEST(AStarCoreTest, DoesNotCutDiagonalCorner)
{
  const int width = 3;
  const int height = 3;

  std::vector<bool> occupied(width * height, false);

  // Block the two cells beside the diagonal.
  occupied[0 * width + 1] = true;
  occupied[1 * width + 0] = true;

  AStarCore astar(width, height, 1.0, occupied);

  const auto path = astar.plan(0, 0, 1, 1);

  EXPECT_TRUE(path.empty());
}
