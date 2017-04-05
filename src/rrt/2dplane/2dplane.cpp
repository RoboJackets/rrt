
#include <math.h>
#include <functional>
#include <rrt/2dplane/2dplane.hpp>

using namespace Eigen;
using namespace RRT;
using namespace std;

const int dimensions = 2;

Tree<Vector2f>* RRT::TreeFor2dPlane(shared_ptr<StateSpace<Eigen::Vector2f>> stateSpace,
    Vector2f goal,
    float step) {
    Tree<Vector2f> *rrt = new Tree<Vector2f>(stateSpace, dimensions, hash);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}
