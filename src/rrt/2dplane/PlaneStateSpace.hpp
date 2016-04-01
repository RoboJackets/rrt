#pragma once

#include <rrt/StateSpace.hpp>
#include <Eigen/Dense>

namespace RRT {

/**
 * @brief A 2d plane with continuous states and no obstacles.
 */
class PlaneStateSpace : public StateSpace<Eigen::Vector2f> {
public:
    PlaneStateSpace(float width, float height);

    Eigen::Vector2f randomState() const;

    Eigen::Vector2f intermediateState(const Eigen::Vector2f &source, const Eigen::Vector2f &target, float stepSize) const;

    double distance(const Eigen::Vector2f &from, const Eigen::Vector2f &to) const;

    /**
     * Returns a boolean indicating whether the given point is within bounds.
     */
    bool stateValid(const Eigen::Vector2f &pt) const;

    float width() const;
    float height() const;


private:
    float _width, _height;
};

}  // namespace RRT
