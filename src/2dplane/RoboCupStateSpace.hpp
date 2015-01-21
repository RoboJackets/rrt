#pragma once

#include <StateSpace.hpp>
#include <2dplane/ObstacleGrid.hpp>
#include <Eigen/Dense>


class RoboCupRobotState {
public:
    Eigen::Vector2f pos, vel;
    RoboCupRobotState(float x, float y, float vx, float vy) : pos(x, y), vel(x, y) {}
    RoboCupRobotState() {}
};


class RoboCupStateSpace : StateSpace<RoboCupRobotState> {
public:
    RoboCupStateSpace(float width, float height, int discretizedWidth, int discretizedHeight, float maxSpeed, float maxAccel);

    ObstacleGrid &obstacleGrid();
    const ObstacleGrid &obstacleGrid() const;


    RoboCupRobotState randomState() const;

    RoboCupRobotState intermediateState(const RoboCupRobotState &source, const RoboCupRobotState &target, float stepSize, bool reverse = false) const;

    /**
     * @brief Weighted sum of normalized position and velocity difference
     */
    double distance(const RoboCupRobotState &from, const RoboCupRobotState &to) const;

    bool positionInBounds(const Eigen::Vector2f &pt) const;
    bool stateValid(const RoboCupRobotState &state) const;

    bool transitionValid(const RoboCupRobotState &from, const RoboCupRobotState &to) const;


private:
    ObstacleGrid _obstacleGrid;

    float _maxSpeed, _maxAccel;
};
