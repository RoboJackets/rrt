#pragma once

#include <rrt/2dplane/PlaneStateSpace.hpp>
#include <rrt/2dplane/ObstacleGrid.hpp>
#include <Eigen/Dense>

namespace RRT {

/**
 * @brief A 2d plane with continuous states and discretized obstacles.
 * @details The state space is broken up into a grid with the given discrete
 * height and widths.
 */
class GridStateSpace : public PlaneStateSpace<Eigen::Vector2f> {
public:
  GridStateSpace(float width, float height, int discretizedWidth,
                 int discretizedHeight);

  /**
   * Returns a boolean indicating whether the given point is within bounds and
   * obstacle-free.
   */
  bool stateValid(const Eigen::Vector2f &pt) const;
  bool transitionValid(const Eigen::Vector2f &from,
                       const Eigen::Vector2f &to) const;

  Eigen::Vector2f intermediateState(const Eigen::Vector2f &source,
                                    const Eigen::Vector2f &target,
                                    float minStepSize, float maxStepSize) const;

  const ObstacleGrid &obstacleGrid() const;
  ObstacleGrid &obstacleGrid();

private:
  ObstacleGrid _obstacleGrid;
};

} // namespace RRT
