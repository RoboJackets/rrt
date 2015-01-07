
#include <Tree.hpp>
#include <StateSpace.hpp>
#include <Eigen/Dense>


/**
 * This creates an instance of an RRT Tree with the callbacks
 * configured from the given parameters.
 *
 * @param w, h The dimensions of the 2d plane.  These are used when
 * picking random points for the Tree to move towards.
 
 * @param goal The point representing the goal that the tree is
 * trying to find a path to
 *
 * @param step The fixed step size that the tree uses.  This is the
 * maximum distance between nodes.
 *
 * @return An RRT::Tree with its callbacks and parameters configured.
 * You'll probably want to override the transitionValidator callback
 * if your 2d plane has any obstacles.
 */
RRT::Tree<Eigen::Vector2f> *TreeFor2dPlane(
    shared_ptr<StateSpace<Eigen::Vector2f>> stateSpace,
    Eigen::Vector2f goal,
    float step);
