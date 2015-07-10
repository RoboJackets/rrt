#include "gtest/gtest.h"
#include <Tree.hpp>
#include <2dplane/GridStateSpace.hpp>
#include <vector>

using namespace RRT;
using namespace Eigen;
using namespace std;

// An easy rrt problem to solve in the GridStateSpace
unique_ptr<Tree<Vector2f>> CreateDefaultTree() {
	auto stateSpace = make_shared<GridStateSpace>(50, 50, 50, 50);
	unique_ptr<Tree<Vector2f>> rrt(new Tree<Vector2f>(stateSpace));
	rrt->setStepSize(0.5);
	rrt->setMaxIterations(10000);
	rrt->setGoalMaxDist(5);
	rrt->setStartState(Vector2f(10, 10));
	rrt->setGoalState(Vector2f(1, 1));
	return rrt;
}

TEST(Tree, Example_2dplane) {
	auto rrt = CreateDefaultTree();
	EXPECT_TRUE(rrt->run());
}

TEST(Tree, GetPath) {
	auto rrt = CreateDefaultTree();
	EXPECT_TRUE(rrt->run());

	//	get path in reverse order (end -> root)
	vector<Vector2f> path;
	rrt->getPath(path, rrt->lastNode(), true);
	EXPECT_TRUE(path.size() > 1);

	//	get path in regular order (root -> end)
	path.clear();
	rrt->getPath(path, rrt->lastNode(), false);
	EXPECT_TRUE(path.size() > 1);
}
