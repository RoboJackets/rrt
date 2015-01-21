#include "RoboCupStateSpace.hpp"
#include <math.h>

using namespace Eigen;


RoboCupStateSpace::RoboCupStateSpace(float width, float height, int discretizedWidth, int discretizedHeight, float maxSpeed, float maxAccel):
    _obstacleGrid(width, height, discretizedWidth, discretizedHeight)
{
    _maxSpeed = maxSpeed;
    _maxAccel = maxAccel;
}

ObstacleGrid &RoboCupStateSpace::obstacleGrid() {
    return _obstacleGrid;
}

const ObstacleGrid &RoboCupStateSpace::obstacleGrid() const {
    return _obstacleGrid;
}

bool RoboCupStateSpace::positionInBounds(const Vector2f &pt) const {
    return pt.x() > 0 && pt.y() > 0 && pt.x() < _obstacleGrid.width() && pt.y() < _obstacleGrid.height();
}

RoboCupRobotState RoboCupStateSpace::randomState() const {
    RoboCupRobotState state;
    state.pos = Vector2f(drand48() * _obstacleGrid.width(), drand48() * _obstacleGrid.height());

    float speed = drand48() * _maxSpeed;
    float angle = drand48() * 2 * M_PI;
    state.vel = Vector2f(speed*cosf(angle), speed*sinf(angle));

    return state;
}

RoboCupRobotState RoboCupStateSpace::intermediateState(const RoboCupRobotState &source, const RoboCupRobotState &target, float stepSize, bool reverse) const {
    //  swap order of states if we're growing the reverse tree
    RoboCupRobotState src = reverse ? target : source;
    RoboCupRobotState dst = reverse ? source : target;

    stepSize /= 30;

    float t = 2.0 * (dst.pos - src.pos).norm() / (dst.vel + src.vel).norm();
    Vector2f a = (dst.vel - src.vel) / t;

    if (a.norm() > _maxAccel) {
        return RoboCupRobotState(-1, -1, -1, -1);
    } else {
        RoboCupRobotState state;
        state.pos = 0.5*a*t*t + src.vel*stepSize + src.pos;
        state.vel = a*t + src.vel;

        return state;
    }
}

double RoboCupStateSpace::distance(const RoboCupRobotState &from, const RoboCupRobotState &to) const {
    Vector2f ds = to.pos - from.pos;
    Vector2f dv = to.vel - from.vel;

    //  normalize by dividing by max values
    dv /= _maxSpeed;
    ds /= sqrtf(powf(_obstacleGrid.width(), 2) + powf(_obstacleGrid.height(), 2));

    //  weighted sum of differences.  note: weights were chosen arbitrarily and should be experimented with
    return 0.3*dv.norm() + 0.7*ds.norm();
}

bool RoboCupStateSpace::stateValid(const RoboCupRobotState &state) const {
    return positionInBounds(state.pos) && !_obstacleGrid.obstacleAt(_obstacleGrid.gridSquareForLocation(state.pos));
}

bool RoboCupStateSpace::transitionValid(const RoboCupRobotState &from, const RoboCupRobotState &to) const {
    return true;    //  TODO: obstacles
}
