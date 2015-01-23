#include "AngleLimitedStateSpace.hpp"
#include <math.h>
#include <cfloat>

using namespace Eigen;


const float MaxAngleDiff = M_PI / 8.0;



float fixAngleRadians(float angle) {
    //  normalize
    while (angle > M_PI) angle -= 2.0*M_PI;
    while (angle < -M_PI) angle += 2.0*M_PI;
    return angle;
}


AngleLimitedStateSpace::AngleLimitedStateSpace(float width, float height, float discretizedWidth, float discretizedHeight) :
    _obstacleGrid(width, height, discretizedWidth, discretizedHeight)
{

}

AngleLimitedState AngleLimitedStateSpace::randomState() const {
    //  note that the generated state has no angle set (its angle will be determined later based on its position relative to another state)
    AngleLimitedState state;
    state.setPos(Vector2f(drand48() * width(), drand48() * height()));
    return state;
}

AngleLimitedState AngleLimitedStateSpace::intermediateState(const AngleLimitedState &source, const AngleLimitedState &target, float stepSize, bool reverse) const {
    Vector2f delta = (target.pos() - source.pos()).normalized();    //  unit vector
    Vector2f newPos = source.pos() + delta * stepSize;
    float newAngle = atan2(delta.y(), delta.x());
    if (reverse) newAngle += M_PI;


    if (fabs(newAngle - source.angle()) > MaxAngleDiff) {
        newAngle = source.angle() + MaxAngleDiff*0.99999 * (newAngle-source.angle() > 0 ? 1 : -1);
        newPos = source.pos() + Vector2f(-cosf(newAngle), -sinf(newAngle));
    }


    return AngleLimitedState(newPos, newAngle, true);
}

double AngleLimitedStateSpace::distance(const AngleLimitedState &from, const AngleLimitedState &to, bool reverse) const {
    Vector2f diff = to.pos() - from.pos();



    float angleDiff = (from.hasAngle() && to.hasAngle()) ? fabs(to.angle() - from.angle()) : 0;

    if (!to.hasAngle()) {
        float angle2 = atan2f(diff.y(), diff.x());
        if (reverse) angle2 += M_PI;
        angleDiff = fabs(angle2 - from.angle());
    }


    float maxDist = sqrtf(width()*width() + height()*height());

    return angleDiff > MaxAngleDiff ? diff.norm() + maxDist: diff.norm();


    const float DistanceWeight = 0.3;
    const float AngleDiffWeight = 1.0 - DistanceWeight;
    // return (diff.norm() / maxDist)*DistanceWeight + (angleDiff / M_PI)*AngleDiffWeight;
}

bool AngleLimitedStateSpace::stateValid(const AngleLimitedState &state) const {
    return _obstacleGrid.pointInBounds(state.pos());
}

bool AngleLimitedStateSpace::transitionValid(const AngleLimitedState &from, const AngleLimitedState &to, bool reverse) const {
    float angleDiff = from.hasAngle() && to.hasAngle() ? fabs(from.angle() - to.angle()) : 0;
    return _obstacleGrid.transitionValid(from.pos(), to.pos()) && angleDiff < MaxAngleDiff;
}

const ObstacleGrid &AngleLimitedStateSpace::obstacleGrid() const {
    return _obstacleGrid;
}

ObstacleGrid &AngleLimitedStateSpace::obstacleGrid() {
    return _obstacleGrid;
}

float AngleLimitedStateSpace::width() const {
    return _obstacleGrid.width();
}

float AngleLimitedStateSpace::height() const {
    return _obstacleGrid.height();
}
