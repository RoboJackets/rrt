#include "AngleLimitedStateSpace.hpp"
#include <planning/Path.hpp>

#include <cmath>
#include <cfloat>
#include <algorithm>
#include <iostream>

using namespace Eigen;
using namespace std;
using std::min;
using std::abs;

ostream &operator<<(ostream &os, const AngleLimitedState &st) {
    os << "AngleLimitedState: pos=(" << st.pos().x() << ", " << st.pos().y()
       << "); angle=" << st.angle() << "; maxAngleDiff=" << st.maxAngleDiff()
       << "; hasAngle=" << st.hasAngle() << "; reverse=" << st.reverse();
    return os;
}

AngleLimitedStateSpace::AngleLimitedStateSpace(float width, float height,
                                               float discretizedWidth,
                                               float discretizedHeight)
    : _obstacleGrid(width, height, discretizedWidth, discretizedHeight) {
    setMaxAngleDiffDecay(1.1);
}

AngleLimitedState AngleLimitedStateSpace::randomState() const {
    //  note that the generated state has no angle set (its angle will be
    //  determined later based on its position relative to another state)
    AngleLimitedState state;
    state.setPos(Vector2f(drand48() * width(), drand48() * height()));
    return state;
}

AngleLimitedState AngleLimitedStateSpace::intermediateState(
    const AngleLimitedState &source, const AngleLimitedState &target,
    float stepSize, bool reverse) const {
    if (target.hasAngle())
        throw std::invalid_argument("target state must be angle-less");

    Vector2f dir = (target.pos() - source.pos()).normalized();  // unit vector
    Vector2f newPos = source.pos() + dir * stepSize;

    // cout << "intermediateState(reverse=" << reverse << ")";

    float prevAngle =
        reverse ? fixAngleRadians(source.angle() + M_PI) : source.angle();

    float newAngle = atan2f(dir.y(), dir.x());

    //  if this new intermediate state would violate the max angle rule, we
    //  rotate it so that it falls just within our constraints.
    //  this goes a long ways towards allowing the rrt to explore as much area
    //  as possible rather than getting stuck because of its angle restrictions
    if (abs(fixAngleRadians(newAngle - prevAngle)) > source.maxAngleDiff()) {
        // cout << "\n\toldAngle too big: " << newAngle;
        newAngle = fixAngleRadians(prevAngle +
                                   source.maxAngleDiff() * 0.98 *
                                       (prevAngle < newAngle ? 1 : -1));
        newPos =
            source.pos() +
            Vector2f(cosf(newAngle), sinf(newAngle)).normalized() * stepSize;
    }

    newAngle =
        reverse ? fixAngleRadians(newAngle + M_PI) : fixAngleRadians(newAngle);

    AngleLimitedState newState(newPos, newAngle, true);
    newState.setMaxAngleDiff(min(newState.maxAngleDiff(),
                                 source.maxAngleDiff() * maxAngleDiffDecay()));

    newState.setReverse(reverse);

    // cout << "\n\tfrom=" << source << "\n\tto=" << target << "\n\tresult=" <<
    // newState << endl;

    return newState;
}

float AngleLimitedStateSpace::distance(const AngleLimitedState &from,
                                       const AngleLimitedState &to) const {

#warning this method doesn't respect the @reverse property

    Vector2f diff = to.pos() - from.pos();
    float angleDiff = (from.hasAngle() && to.hasAngle())
                          ? abs(fixAngleRadians(to.angle() - from.angle()))
                          : 0;

    //  if @to doesn't have an angle set, we calculate it based on the angle of
    //  the vector @from->@to
    if (!to.hasAngle()) {
        float angle2 = atan2f(diff.y(), diff.x());
        angleDiff = abs(fixAngleRadians(angle2 - from.angle()));
    }

    //  if it's above maxAngleDiff, it's distance metric is in a second tier
    //  the first tier is maxDist less than the second tier in value
    float maxDist = sqrtf(width() * width() + height() * height());
    return angleDiff > from.maxAngleDiff() ? diff.norm() + maxDist
                                           : diff.norm();
}

bool AngleLimitedStateSpace::stateValid(const AngleLimitedState &state) const {
    return _obstacleGrid.pointInBounds(state.pos());
}

bool AngleLimitedStateSpace::transitionValid(
    const AngleLimitedState &from, const AngleLimitedState &to) const {
    // float maxAngleDiff = min(from.maxAngleDiff(), to.maxAngleDiff());
    // float angleDiff = from.hasAngle() && to.hasAngle() ?
    // abs(fixAngleRadians(from.angle() - to.angle())) : 0;

    // check for obstacles
    if (!_obstacleGrid.transitionValid(from.pos(), to.pos())) return false;

    //  Calculate angle between states and ensure it's within the maxAngleDiff
    //  constraints of both @from and @to, taking into account the @reverse
    //  property.
    Vector2f diff = to.pos() - from.pos();
    float angle2 = atan2f(diff.y(), diff.x());

    if (from.hasAngle()) {
        float angleDiff = from.reverse() ? from.angle() - angle2 + M_PI
                                         : from.angle() - angle2;
        angleDiff = fixAngleRadians(angleDiff);
        if (abs(angleDiff) > from.maxAngleDiff()) return false;
    }

    if (to.hasAngle()) {
        float angleDiff =
            to.reverse() ? to.angle() - angle2 + M_PI : to.angle() - angle2;
        angleDiff = fixAngleRadians(angleDiff);
        if (abs(angleDiff) > to.maxAngleDiff()) return false;
    }

    return true;
}

const ObstacleGrid &AngleLimitedStateSpace::obstacleGrid() const {
    return _obstacleGrid;
}

ObstacleGrid &AngleLimitedStateSpace::obstacleGrid() { return _obstacleGrid; }

float AngleLimitedStateSpace::width() const { return _obstacleGrid.width(); }

float AngleLimitedStateSpace::height() const { return _obstacleGrid.height(); }

float AngleLimitedStateSpace::maxAngleDiffDecay() const {
    return _maxAngleDiffDecay;
}

void AngleLimitedStateSpace::setMaxAngleDiffDecay(float decay) {
    _maxAngleDiffDecay = decay;
}

void AngleLimitedStateSpace::PathModifier(std::vector<AngleLimitedState> &states,
                                     int start, int end) {
    // Use the default implementation to remove the intermediate states
    Planning::DefaultPathModifier<AngleLimitedState>(states, start, end);

    // TODO: update angles!!!!!!!!

    #warning unimplemented!
}
