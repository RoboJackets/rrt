
#include "2dplane.hpp"
#include <functional>
#include <math.h>

using namespace Eigen;
using namespace RRT;


// bool pointInRect(const Rect &rect, const Vector2f &pt) {
//  return pt.x() > rect.origin.x() && pt.x() < rect.origin.x() + rect.w
//          && pt.y() > rect.origin.y() && pt.y() < rect.origin.y() + rect.h;
// }

float magnitude(Vector2f &vec) {
    return sqrtf( powf(vec.x(), 2) + powf(vec.y(), 2) );
}

Tree<Vector2f> *TreeFor2dPlane(float w, float h, Vector2f goal, float step) {
    Tree<Vector2f> *rrt = new Tree<Vector2f>();

    rrt->setStepSize(step);

    //  pick a random point within the plane
    rrt->randomStateGenerator = [=]() {
        return Vector2f(drand48() * w, drand48() * h);
    };

    //  distance
    rrt->distanceCalculator = [](const Vector2f &stateA, const Vector2f &stateB) {
        Vector2f delta = stateB - stateA;
        return magnitude(delta);
    };

    rrt->transitionValidator = [=](const Vector2f &start, const Vector2f &newState) {
        return true;
    };

    rrt->intermediateStateGenerator = [=](const Vector2f &source, const Vector2f &target, float step) {
        Vector2f delta = target - source;
        delta = delta / magnitude(delta);   //  unit vector

        Vector2f val = source + delta * step;
        return val;
    };

    rrt->setGoalState(goal);

    return rrt;
}
