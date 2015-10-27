#include "gtest/gtest.h"
#include <Tree.hpp>
#include <2dplane/2dplane.hpp>
#include <2dplane/GridStateSpace.hpp>
#include <vector>

using namespace RRT;
using namespace Eigen;
using namespace std;


TEST(Tree, Example_2dplane) {
	Tree<Vector2f> *tree = TreeFor2dPlane(
		make_shared<GridStateSpace>(50, 50, 50, 50),
		Vector2f(40, 40),	//	goal point
		5);					//	step size

	//	give it plenty of iterations so it's not likely to fail
	const int maxIterations = 10000;
	tree->setMaxIterations(maxIterations);
	tree->setGoalMaxDist(5);

	tree->setStartState(Vector2f(10, 10));
	bool success = tree->run();	//	run with the given starting point
	ASSERT_TRUE(success);
}

TEST(Tree, GetPath) {
	Tree<Vector2f> *tree = TreeFor2dPlane(
		make_shared<GridStateSpace>(50, 50, 50, 50),
		Vector2f(40, 40),	//	goal point
		5);					//	step size

	//	give it plenty of iterations so it's not likely to fail
	const int maxIterations = 10000;
	tree->setMaxIterations(maxIterations);
	tree->setGoalMaxDist(5);

	tree->setStartState(Vector2f(10, 10));
	bool success = tree->run();	//	run with the given starting point
	ASSERT_TRUE(success);

	//	get path in reverse order (end -> root)
	vector<Vector2f> path;
	tree->getPath(path, tree->lastNode(), true);
	ASSERT_TRUE(path.size() > 1);

	//	get path in regular order (root -> end)
	path.clear();
	tree->getPath(path, tree->lastNode(), false);
	ASSERT_TRUE(path.size() > 1);
}
