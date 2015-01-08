#pragma once

#include <2dplane/PlaneStateSpace.hpp>
#include <Eigen/Dense>


/**
 * @brief A 2d plane with continuous states and discretized obstacles.
 * @details The state space is broken up into a grid with the given discrete height and widths.
 */
class GridStateSpace : public PlaneStateSpace {
public:
    GridStateSpace(float width, float height, int discretizedWidth, int discretizedHeight);
    ~GridStateSpace();

    /**
     * Returns a boolean indicating whether the given point is within bounds and obstacle-free.
     */
    bool stateValid(const Eigen::Vector2f &pt) const;
    bool transitionValid(const Eigen::Vector2f &from, const Eigen::Vector2f &to) const;

    Eigen::Vector2i gridSquareForState(const Eigen::Vector2f &state) const;

    void clearObstacles();
    bool &obstacleAt(int x, int y);
    bool obstacleAt(int x, int y) const;
    bool &obstacleAt(const Eigen::Vector2i &gridLoc);
    bool obstacleAt(const Eigen::Vector2i &gridLoc) const;

    int discretizedWidth() const;
    int discretizedHeight() const;


private:
    int _discretizedWidth, _discretizedHeight;

    /// 2d array of obstacles
    bool *_obstacles;
};
