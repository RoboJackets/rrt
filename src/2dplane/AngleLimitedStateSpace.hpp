#pragma once

#include <StateSpace.hpp>
#include <2dplane/ObstacleGrid.hpp>
#include <Eigen/Dense>
#include <util.hpp>

class AngleLimitedState {
   public:
    AngleLimitedState(const Eigen::Vector2f &pos = Eigen::Vector2f(0, 0),
                      float angle = 0, bool hasAngle = false) {
        setPos(pos);
        setAngle(angle);
        setHasAngle(hasAngle);
        setMaxAngleDiff(M_PI / 6.0);
    }

    void setAngle(float angle) { _angle = fixAngleRadians(angle); }
    float angle() const { return _angle; }

    void setPos(const Eigen::Vector2f &pos) { _pos = pos; }
    const Eigen::Vector2f &pos() const { return _pos; }

    void setHasAngle(bool hasAngle) { _hasAngle = hasAngle; }
    bool hasAngle() const { return _hasAngle; }

    float maxAngleDiff() const { return _maxAngleDiff; }
    void setMaxAngleDiff(float maxAngleDiff) { _maxAngleDiff = maxAngleDiff; }

   private:
    Eigen::Vector2f _pos;
    float _angle;
    float _maxAngleDiff;
    bool _hasAngle;
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

    float distance(const AngleLimitedState &from,
                   const AngleLimitedState &to) const;

    bool stateValid(const AngleLimitedState &state) const;
    bool transitionValid(const AngleLimitedState &from,
                         const AngleLimitedState &to) const;

    const ObstacleGrid &obstacleGrid() const;
    ObstacleGrid &obstacleGrid();

    float width() const;
    float height() const;

    /**
     * @brief How much the maxAngleDiff increases for each subsequent node.
     * Note that this increase is limited to a max value.
     * @details This essentially lets us increase the allowable curvature of the
     * path slowly as the tree leads away from its startpoint.
     */

    // TODO: update docs, this is now a multiplier!
    // TODO: rewrite this in terms of curvature, so it's step-size independent
    // angleDiff*decay = newAngleDiff
    float maxAngleDiffDecay() const;
    void setMaxAngleDiffDecay(float decay);

    /// A function for use with SmoothPath() that deletes states between (but
    /// not including) the start and end indexes and adjusts the angle property
    /// of the start state.
    static void PathModifier(std::vector<AngleLimitedState> &states, int start,
                        int end);


    // static float MaxAngleDiffForCurvature(float curvatureLimit, float stepSize);

    // /// Same as above, but uses @accelLimit to calculate the curvature limit
    // static float MaxAngleDiffForSpeed(float speed, float accelLimit,
    //                                   float stepSize);

   private:
    ObstacleGrid _obstacleGrid;
    float _maxAngleDiffDecay;
};
