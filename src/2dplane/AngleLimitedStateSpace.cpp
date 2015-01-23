#include "AngleLimitedStateSpace.hpp"
#include <math.h>
#include <cfloat>

using namespace Eigen;


const float MaxAngleDiff = M_PI / 8.0;


AngleLimitedStateSpace::AngleLimitedStateSpace(float width, float height, float discretizedWidth, float discretizedHeight) :
    _obstacleGrid(width, height, discretizedWidth, discretizedHeight)
{

}

AngleLimitedState AngleLimitedStateSpace::randomState() const {
    //  note that the generated has no angle set
    AngleLimitedState state;
    state.setPos(Vector2f(drand48() * width(), drand48() * height()));
    return state;
}

AngleLimitedState AngleLimitedStateSpace::intermediateState(const AngleLimitedState &source, const AngleLimitedState &target, float stepSize, bool reverse) const {
    Vector2f delta = (target.pos() - source.pos()).normalized();    //  unit vector
    Vector2f newPos = source.pos() + delta * stepSize;
    float newAngle = atan2(delta.y(), delta.x());
    if (reverse) newAngle += M_PI;
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


    const float DistanceWeight = 0.3;
    const float AngleDiffWeight = 1.0 - DistanceWeight;

    float maxDist = sqrtf(width()*width() + height()*height());

    return angleDiff > MaxAngleDiff ? FLT_MAX : diff.norm();

    return (diff.norm() / maxDist)*DistanceWeight + (angleDiff / M_PI)*AngleDiffWeight;
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
