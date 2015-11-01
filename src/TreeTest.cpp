#include "gtest/gtest.h"
#include <Tree.hpp>
#include <2dplane/2dplane.hpp>
#include <2dplane/GridStateSpace.hpp>
#include <vector>

#define MAGNITUDE(n) (sqrtf(powf(n.x(), 2) + powf(n.y(), 2)))

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

TEST(Tree, FailOnImpossibleRequest) {
	Tree<Vector2f> *tree = TreeFor2dPlane(
		make_shared<GridStateSpace>(50, 50, 50, 50),
		Vector2f(60, 60),	//	goal point outside the bounds of the state space
		5);					//	step size

	//	give it plenty of iterations so it's not likely to fail
	const int maxIterations = 2000;
	tree->setMaxIterations(maxIterations);
	tree->setGoalMaxDist(5);

	tree->setStartState(Vector2f(10, 10));
	bool success = tree->run();	//	run with the given starting point
	ASSERT_FALSE(success); // the rrt search should fail because the goal isn't reachable
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

TEST(Tree, ASC) {
	//test adaptive stepsize control
	Tree<Vector2f> *tree = TreeFor2dPlane(
		make_shared<GridStateSpace>(50, 50, 50, 50),
		Vector2f(40, 40),	//	goal point
		5);					//	step size

	//	give it plenty of iterations so it's not likely to fail
	const int maxIterations = 10000;
	tree->setMaxIterations(maxIterations);
	tree->setGoalMaxDist(5);
	tree->setASCEnabled(true);

	tree->setStartState(Vector2f(10, 10));
	bool success = tree->run();	//	run with the given starting point
	ASSERT_TRUE(success);

	vector<Vector2f> path;
	tree->getPath(path, tree->lastNode(), true);

	bool varied = false;
	for (int i = 1; !varied && i < path.size() - 2; i++) {
		Vector2f x = path[i] - path[i - 1];
		Vector2f y = path[i] - path[i + 1];
		float n = MAGNITUDE(x) / MAGNITUDE(y);
		if (n < 0.99 || n > 1.01) varied = true;
	}
	ASSERT_TRUE(varied);
}
