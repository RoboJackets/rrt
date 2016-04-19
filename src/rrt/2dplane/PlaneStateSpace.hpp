#pragma once

#include <rrt/StateSpace.hpp>
#include <Eigen/Dense>

namespace RRT {

/**
 * @brief A 2d plane with continuous states and no obstacles.
 */
template <class POINT_CLASS = Eigen::Vector2f>
class PlaneStateSpace : public StateSpace<POINT_CLASS> {
public:
    PlaneStateSpace(float width, float height)
        : _width(width), _height(height) {}

    POINT_CLASS randomState() const {
        return POINT_CLASS(drand48() * width(), drand48() * height());
    }

    POINT_CLASS intermediateState(const POINT_CLASS& source,
                                  const POINT_CLASS& target,
                                  float stepSize) const {
        POINT_CLASS delta = target - source;
        delta = delta / delta.norm();  //  unit vector

        POINT_CLASS val = source + delta * stepSize;
        return val;
    }

    double distance(const POINT_CLASS& from, const POINT_CLASS& to) const {
        POINT_CLASS delta = from - to;
        return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
    }

    /**
     * Returns a boolean indicating whether the given point is within bounds.
     */
    bool stateValid(const POINT_CLASS& pt) const {
        return pt.x() >= 0 && pt.y() >= 0 && pt.x() < width() &&
               pt.y() < height();
    }

    float width() const { return _width; }
    float height() const { return _height; }

private:
    float _width, _height;
};

}  // namespace RRT
