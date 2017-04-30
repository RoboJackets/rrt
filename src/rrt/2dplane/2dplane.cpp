
#include <math.h>
#include <functional>
#include <rrt/2dplane/2dplane.hpp>

using namespace Eigen;
using namespace RRT;
using namespace std;

Tree<Vector2d>* RRT::TreeFor2dPlane(shared_ptr<StateSpace<Eigen::Vector2d>> stateSpace,
    Vector2d goal,
    double step) {
    Tree<Vector2d> *rrt = new Tree<Vector2d>(stateSpace, dimensions, hash);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}
