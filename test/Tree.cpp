#include "gtest/gtest.h"
#include <Tree.hpp>
#include <2dplane/2dplane.hpp>
#include <vector>

using namespace RRT;
using namespace Eigen;
using namespace std;


TEST(Instantiation, Tree) {
	Tree<int> *tree = new Tree<int>();
}

TEST(Example_2dplane, Tree) {
	Tree<Vector2f> *tree = TreeFor2dPlane(
		100,				//	width
		100,				//	height
		Vector2f(90, 90),	//	goal point
		5);					//	step size

	//	give it plenty of iterations so it's not likely to fail
	const int maxIterations = 10000;
	tree->setMaxIterations(maxIterations);

	bool success = tree->run(Vector2f(10, 10));	//	run with the given starting point

	ASSERT_EQ(success, true);
}

TEST(GetPath, Tree) {
	Tree<Vector2f> *tree = TreeFor2dPlane(
		100,				//	width
		100,				//	height
		Vector2f(90, 90),	//	goal point
		5);					//	step size

	//	give it plenty of iterations so it's not likely to fail
	const int maxIterations = 10000;
	tree->setMaxIterations(maxIterations);

	bool success = tree->run(Vector2f(10, 10));	//	run with the given starting point

	//	get path in reverse order (end -> root)
	vector<Vector2f> path;
	tree->getPath(path, tree->lastNode(), true);
	ASSERT_EQ(true, path.size() > 1);

	//	get path in regular order (root -> end)
	path.clear();
	tree->getPath(path, tree->lastNode(), false);
	ASSERT_EQ(true, path.size() > 1);
}
