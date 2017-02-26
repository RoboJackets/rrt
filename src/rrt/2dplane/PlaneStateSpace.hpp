#pragma once

#include <Eigen/Dense>
#include <rrt/StateSpace.hpp>

#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

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

    // POINT_CLASS randomBiasState(const POINT_CLASS& goalState, float goalBias) const {
    POINT_CLASS randomBiasState(const POINT_CLASS& goalState, float goalBias) const {
        // ensures that randX and randY are within the fields bounds
        int randX = std::max(std::min(goalState.x() + logit(goalBias, drand48()), width() / 2), width() / 2 * -1);
        cout << width() << endl;
        int randY = std::max(std::min(goalState.y() + logit(goalBias, drand48()), height() / 2), height() / 2 * -1);
        return POINT_CLASS(randX, randY);
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

    float logit(const float goalBias, const float num) const{
        return goalBias * 2000 * log(num/(1 - num));
    }

    float width() const { return _width; }
    float height() const { return _height; }

private:
    float _width, _height;
};

}  // namespace RRT
