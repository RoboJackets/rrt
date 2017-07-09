#pragma once

#include <Eigen/Dense>

namespace RRT {

/**
 * @brief Handles a grid of obstacles laid over a continuous 2d plane.
 * @details The state space is broken up into a grid with the given discrete
 * height and widths.
 */
class ObstacleGrid {
public:
    ObstacleGrid(double width, double height, int discretizedWidth,
                 int discretizedHeight);
    ~ObstacleGrid();

    Eigen::Vector2i gridSquareForLocation(const Eigen::Vector2d& loc) const;

    /**
     * Finds the distance from state to its neareset obstacle. Only searches up
     *to
     * maxDist around
     * state so as to not waste time checking far away and irrelevant obstacles.
     *
     * @param state The location to search with respect to for the nearest
     * obstacle dist
     * @param maxDist The maximum vertical and horizontal distance from state to
     * search for obstacles
     */
    double nearestObstacleDist(const Eigen::Vector2d& state,
                              double maxDist) const;
    void clear();
    bool& obstacleAt(int x, int y);
    bool obstacleAt(int x, int y) const;
    bool& obstacleAt(const Eigen::Vector2i& gridLoc);
    bool obstacleAt(const Eigen::Vector2i& gridLoc) const;

    int discretizedWidth() const;
    int discretizedHeight() const;
    double width() const;
    double height() const;

private:
    int _discretizedWidth, _discretizedHeight;
    double _width, _height;

    /// 2d array of obstacles
    bool* _obstacles;
};

}  // namespace RRT
