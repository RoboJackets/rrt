
#include "2dplane.hpp"
#include <functional>
#include <cmath>

using namespace Eigen;
using namespace RRT;
using namespace std;


Tree<Vector2f> *TreeFor2dPlane(shared_ptr<StateSpace<Eigen::Vector2f>> stateSpace,
    Vector2f goal,
    float step) {
    Tree<Vector2f> *rrt = new Tree<Vector2f>(stateSpace);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}
