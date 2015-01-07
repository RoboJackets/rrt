
#include "2dplane.hpp"
#include <functional>
#include <math.h>

using namespace Eigen;
using namespace RRT;


// bool pointInRect(const Rect &rect, const Vector2f &pt) {
//  return pt.x() > rect.origin.x() && pt.x() < rect.origin.x() + rect.w
//          && pt.y() > rect.origin.y() && pt.y() < rect.origin.y() + rect.h;
// }

Tree<Vector2f> *TreeFor2dPlane(shared_ptr<StateSpace<Eigen::Vector2f>> stateSpace,
    Vector2f goal,
    float step) {
    Tree<Vector2f> *rrt = new Tree<Vector2f>(stateSpace);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}
