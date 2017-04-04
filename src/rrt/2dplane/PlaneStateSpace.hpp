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

    /**
     * Generates a point based on a goal bias and a goal state
     *
     * @params goalState - Point that we want to extend towards
     * @params goalBias - Changes likelihood of choosing a point near goal state
     * @returns POINT_CLASS - random point based on goal bias and goal state
     */
    POINT_CLASS randomBiasState(const POINT_CLASS& goalState, float goalBias) const {
        // Generates random value based on goalBias
        float logitX = logit(goalBias / 3 + .67, (float) rand() / RAND_MAX);
        float logitY = logit(goalBias / 3 + .67, (float) rand() / RAND_MAX);
        float offsetY = 0;
        float offsetX = 0;

        // Scale X value based on distance from border
        offsetX = std::max(goalState.x(), width() - goalState.x()) * logitX;

        // Scale Y value based on distance from border
        offsetY = std::max(goalState.y(), height() - goalState.y()) * logitY;
        
        int randX = goalState.x() + offsetX;
        int randY = goalState.y() + offsetY;
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

    /**
     * Uses the logit function to generate a random value based on a goal bias
     *
     * @params goalBias - increases / decreases distance from 0
     * @params num - value to be inputted into logit function
     * @returns float - output of logit function scaled based on goal bias
     */
    float logit(const float goalBias, const float num) const{
        return (1 - goalBias) * log(num/(1 - num));
    }

    float width() const { return _width; }
    float height() const { return _height; }

private:
    float _width, _height;
};

}  // namespace RRT
