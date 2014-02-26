
#include <Tree.hpp>
#include <Eigen/Dense>


/**
 * Gets the length of an Eigen Vector.  This function probably
 * doesn't belong here, but Eigen doesn't provide this functionality
 * and it's pretty useful.
 */
float magnitude(Eigen::Vector2f &vec);

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
	float w,
	float h,
	Eigen::Vector2f goal,
	float step);
