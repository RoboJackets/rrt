#pragma once

#include <2dplane/ObstacleGrid.hpp>
#include <StateSpace.hpp>
#include <util.hpp>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <vector>

class AngleLimitedState {
 public:
  AngleLimitedState(const Eigen::Vector2f &pos = Eigen::Vector2f(0, 0),
    boost::optional<float> inAngle = boost::none,
    boost::optional<float> outAngle = boost::none)
      : _pos(pos), _inAngle(inAngle), _outAngle(outAngle) {
    setMaxCurvature(3);
  }

  void setPos(const Eigen::Vector2f &pos) { _pos = pos; }
  const Eigen::Vector2f &pos() const { return _pos; }

  const boost::optional<float> &inAngle() const { return _inAngle; }
  boost::optional<float> &inAngle() { return _inAngle; }

  const boost::optional<float> &outAngle() const { return _outAngle; }
  boost::optional<float> &outAngle() { return _outAngle; }

  float maxCurvature() const { return _maxCurvature; }
  void setMaxCurvature(float maxCurvature) { _maxCurvature = maxCurvature; }

 private:
  Eigen::Vector2f _pos;
  boost::optional<float> _inAngle;
  boost::optional<float> _outAngle;
  float _maxCurvature;
};

std::ostream &operator<<(std::ostream &os, const AngleLimitedState &st);

class AngleLimitedStateSpace : public StateSpace<AngleLimitedState> {
 public:
  AngleLimitedStateSpace(float width, float height, float discretizedWidth,
                         float discretizedHeight);

  AngleLimitedState randomState() const;
  AngleLimitedState intermediateState(const AngleLimitedState &source,
                                      const AngleLimitedState &target,
                                      float stepSize,
                                      bool reverse = false) const;

  /// Returns infinity if the angle between @from and @to is beyond the angle
  /// bounds of either state or the distance between the two positions
  /// otherwise.
  // TODO: document "second tier"
  /// note: distance() is NOT commutative
  float distance(const AngleLimitedState &from,
                 const AngleLimitedState &to) const;

  bool stateValid(const AngleLimitedState &state) const;

  /// note: transitionValid() is NOT commutative
  bool transitionValid(const AngleLimitedState &from,
                       const AngleLimitedState &to) const;

  const ObstacleGrid &obstacleGrid() const;
  ObstacleGrid &obstacleGrid();

  float width() const;
  float height() const;

  /// Each subsequent state has a maxCurvature equal to the parent state's
  /// maxCurvature multiplied by this factor.  It is limited to the
  /// maxCurvature defined for the state space.
  float curvatureIncreaseFactor() const {
    return _curvatureIncreaseFactor;
  }
  void setCurvatureIncreaseFactor(float factor) {
    _curvatureIncreaseFactor = factor;
  }

  /// The maximum allowed curvature in this state space.
  float maxCurvature() const { return _maxCurvature; }
  void setMaxCurvature(float diff) { _maxCurvature = diff; }

  /// A function for use with SmoothPath() that deletes states between (but
  /// not including) the start and end indexes and adjusts the angle property
  /// of the start state.
  static void PathModifier(std::vector<AngleLimitedState> &states, int start,
                           int end);

  /// Sets inAngle and outAngle for all states based on their difference in
  /// position.  Doesn't touch inAngle of start or outAngle of end.
  static void RecalculateAngles(std::vector<AngleLimitedState> &states) {
    for (int i = 0; i < states.size()-1; ++i) {
      Eigen::Vector2f diff = states[i+1].pos() - states[i].pos();
      float angle = atan2f(diff.y(), diff.x());
      states[i].outAngle() = angle;
      states[i+1].inAngle() = angle;
    }
  }

  /// Calculates the exterior angle for a regular polygon with the given step
  /// size inscribed in a circle with the given curvature.
  static float CalculateExteriorAngleForCurvature(float curvature,
    float sideLength) {
    // for inscribed polygons, sideLength = 2*radius*sin(pi/numSides)
    // for regular polygons (equal side lengths), numSides*exteriorAngle = 2*pi
    // extAngle = 2*pi/numSides
    // numSides = 2*pi/extAngle
    // sideLength = 2*radius*sin(extAngle/2)
    // radius = 1/curvature
    // sideLength = 2*(1/curvature)*sin(extAngle/2)
    // extAngle = 2*asin(sideLength*curvature/2)
    return 2.0 * asinf(sideLength * curvature / 2.0);
  }

  /// Calculates the maximum that the angles between two states can differ if
  /// they are @stepSize distance apart
  static float CalculateMaxAngleDiff(float speed, float accelLimit,
    float stepSize) {
    // accelCentripetal = speed^2 / radius
    // accelCentripetal = accelLimit
    // radius = speed^2 / accelLimit
    // curvature = 1 / radius = accelLimit / speed^2
    float curvature = accelLimit / powf(speed, 2);
    return CalculateExteriorAngleForCurvature(curvature, stepSize);
  }


 private:
  ObstacleGrid _obstacleGrid;
  float _curvatureIncreaseFactor;
  float _maxCurvature;
};
