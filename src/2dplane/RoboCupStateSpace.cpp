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

    float t = 2.0 * (dst.pos - src.pos).norm() / (dst.vel + src.vel).norm();
    Vector2f a = (dst.vel - src.vel) / t;

    if (a.norm() > _maxAccel) {
        return RoboCupRobotState(-1, -1, -1, -1);
    } else {
        float dt = 0.2;
        RoboCupRobotState state;

        state.pos = 0.5*a*t*t + src.vel*dt + src.pos;
        state.vel = a*t + src.vel;

        return state;
    }

    // //  TODO: apply an acceleration for a fixed time interval

    // //  use the formula Vf^2 = Vi^2 + 2*a*d
    // //  rearrange to get (Vf^2 - Vi^2) / (2*d) = a
    // //  we do this once for x and once for y
    // Vector2f accel(
    //     (powf(target.vel.x(), 2) - powf(source.vel.x(), 2)) / (2*(target.pos.x() - source.pos.x())),
    //     (powf(target.vel.y(), 2) - powf(source.vel.y(), 2)) / (2*(target.pos.y() - source.pos.y()))
    // );

    // if (accel.norm() > _maxAccel) {
    //     //  requires too high of an acceleration, return an invalid state
    //     return RoboCupRobotState(-1,-1,-1,-1);
    // } else {
    //     const float dt = 0.2;
    //     RoboCupRobotState state;

    //     state.pos = 0.5*accel*powf(dt, 2) + source.vel*dt + source.pos;
    //     state.vel = source.vel + accel*dt;

    //     return state;
    // }
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
