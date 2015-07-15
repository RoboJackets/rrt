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
     << ")";

  os << "; inAngle=";
  if (st.inAngle()) {
    os << *st.inAngle();
  } else {
    os << "*";
  }

  os << "; outAngle=";
  if (st.outAngle()) {
    os << *st.outAngle();
  } else {
    os << "*";
  }

  os << "; maxAngleDiff=" << st.maxAngleDiff();

  return os;
}

AngleLimitedStateSpace::AngleLimitedStateSpace(float width, float height,
                                               float discretizedWidth,
                                               float discretizedHeight)
    : _obstacleGrid(width, height, discretizedWidth, discretizedHeight) {
  setMaxAngleDiffDecay(1.1);
  setMaxAngleDiff(M_PI);
}

AngleLimitedState AngleLimitedStateSpace::randomState() const {
  //  note that the generated state has no angle set (its angle will be
  //  determined later based on its position relative to another state)
  Eigen::Vector2f randPos(drand48() * width(), drand48() * height());
  return AngleLimitedState(randPos);
}

AngleLimitedState AngleLimitedStateSpace::intermediateState(
    const AngleLimitedState &source, const AngleLimitedState &target,
    float stepSize, bool reverse) const {
  if (target.inAngle() || target.outAngle())
    throw std::invalid_argument("target state must not have any angles set");

  Vector2f dir = (target.pos() - source.pos()).normalized();  // unit vector
  float newAngle = atan2f(dir.y(), dir.x());
  // if (reverse) newAngle = fixAngleRadians(newAngle + M_PI);

  //  if this new intermediate state would violate the max angle rule, we
  //  rotate it so that it falls just within our constraints.
  //  this goes a long ways towards allowing the rrt to explore as much area
  //  as possible rather than getting stuck because of its angle restrictions
  float sourceAngle =
      reverse ? fixAngleRadians(*source.outAngle() + M_PI) : *source.inAngle();
  float angleDiff = fixAngleRadians(newAngle - sourceAngle);
  if (abs(angleDiff) > source.maxAngleDiff()) {
    newAngle = fixAngleRadians(sourceAngle +
                               source.maxAngleDiff() * 0.99 *
                                   (angleDiff > 0 ? 1 : -1));
  }

  Vector2f newPos = source.pos() +
                    Vector2f(cosf(newAngle), sinf(newAngle)).normalized() *
                        stepSize;

  AngleLimitedState newState(newPos);
  if (reverse) {
    newState.outAngle() = newAngle;
  } else {
    newState.inAngle() = newAngle;
  }
  newState.setMaxAngleDiff(
      min(source.maxAngleDiff() * maxAngleDiffDecay(), _maxAngleDiff));

  return newState;
}

float AngleLimitedStateSpace::distance(const AngleLimitedState &from,
                                       const AngleLimitedState &to) const {
  float dist = (from.pos() - to.pos()).norm();

  // if the angles don't line up, the distance is in a "second tier"
  if (!transitionValid(from, to)) {
    const float max_dist = sqrtf(width() * width() + height() * height());
    dist += max_dist;
  }

  return dist;
}

bool AngleLimitedStateSpace::stateValid(const AngleLimitedState &state) const {
  return _obstacleGrid.pointInBounds(state.pos());
}

// forward -> forward
// reverse -> child reverse in same tree
// forward -> reverse

bool AngleLimitedStateSpace::transitionValid(
    const AngleLimitedState &from, const AngleLimitedState &to) const {
  // check for obstacles
  if (!_obstacleGrid.transitionValid(from.pos(), to.pos())) return false;

  //  Calculate angle between states and ensure it's within the maxAngleDiff
  //  constraints of both @from and @to, taking into account the @reverse
  //  property.
  Vector2f diff = to.pos() - from.pos();
  float newAngle = atan2f(diff.y(), diff.x());

  cout << "diff: (" << diff.x() << ", " << diff.y() << ")" << endl;
  cout << "newAngle: " << newAngle << endl;

  if (from.inAngle()) {
    float angleDiff =
        fixAngleRadians(newAngle - *from.inAngle());
    cout << "angleDiff w/from: " << angleDiff << endl;
    if (abs(angleDiff) > from.maxAngleDiff()) return false;
  }

  if (to.outAngle()) {
    float angleDiff = fixAngleRadians(*to.outAngle() - newAngle);
    cout << "angleDiff w/to: " << angleDiff << endl;
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

void AngleLimitedStateSpace::PathModifier(
    std::vector<AngleLimitedState> &states, int start, int end) {
  // Use the default implementation to remove the intermediate states
  Planning::DefaultPathModifier<AngleLimitedState>(states, start, end);

  Vector2f diff = states[start + 1].pos() - states[start].pos();
  float newAngle = atan2f(diff.y(), diff.x());

  states[start].outAngle() = newAngle;
  states[start+1].inAngle() = newAngle;
}
