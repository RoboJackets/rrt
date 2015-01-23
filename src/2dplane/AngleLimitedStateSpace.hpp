#pragma once

#include <StateSpace.hpp>
#include <2dplane/ObstacleGrid.hpp>
#include <Eigen/Dense>


//  constrains the angle to one between -pi and pi
float fixAngleRadians(float angle);


class AngleLimitedState {
public:
    AngleLimitedState(const Eigen::Vector2f &pos = Eigen::Vector2f(0,0), float angle = 0, bool hasAngle = false) {
        setPos(pos);
        setAngle(angle);
        setHasAngle(hasAngle);
    }

    void setAngle(float angle) {
        _angle = fixAngleRadians(angle);
    }
    float angle() const {
        return _angle;
    }

    void setPos(const Eigen::Vector2f &pos) {
        _pos = pos;
    }
    const Eigen::Vector2f &pos() const {
        return _pos;
    }

    void setHasAngle(bool hasAngle) {
        _hasAngle = hasAngle;
    }
    bool hasAngle() const {
        return _hasAngle;
    }

    float maxAngleDiff;


private:
    Eigen::Vector2f _pos;
    float _angle;
    bool _hasAngle;
};



class AngleLimitedStateSpace : public StateSpace<AngleLimitedState> {
public:
    AngleLimitedStateSpace(float width, float height, float discretizedWidth, float discretizedHeight);

    AngleLimitedState randomState() const;
    AngleLimitedState intermediateState(const AngleLimitedState &source, const AngleLimitedState &target, float stepSize, bool reverse = false) const;

    double distance(const AngleLimitedState &from, const AngleLimitedState &to, bool reverse = false) const;

    bool stateValid(const AngleLimitedState &state) const;
    bool transitionValid(const AngleLimitedState &from, const AngleLimitedState &to, bool reverse = false) const;

    const ObstacleGrid &obstacleGrid() const;
    ObstacleGrid &obstacleGrid();

    float width() const;
    float height() const;

private:
    ObstacleGrid _obstacleGrid;
};
