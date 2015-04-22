#pragma once

#include <Eigen/Dense>


/**
 * @brief Handles a grid of obstacles laid over a continuous 2d plane.
 * @details The state space is broken up into a grid with the given discrete height and widths.
 */
class ObstacleGrid {
public:
    ObstacleGrid(float width, float height, int discretizedWidth, int discretizedHeight);
    ~ObstacleGrid();

    Eigen::Vector2i gridSquareForLocation(const Eigen::Vector2f &loc) const;

    float nearestObstacleDist(const Eigen::Vector2f &state) const;
    void clear();
    bool &obstacleAt(int x, int y);
    bool obstacleAt(int x, int y) const;
    bool &obstacleAt(const Eigen::Vector2i &gridLoc);
    bool obstacleAt(const Eigen::Vector2i &gridLoc) const;

    int discretizedWidth() const;
    int discretizedHeight() const;
    float width() const;
    float height() const;


private:
    int _discretizedWidth, _discretizedHeight;
    float _width, _height;

    /// 2d array of obstacles
    bool *_obstacles;
};
