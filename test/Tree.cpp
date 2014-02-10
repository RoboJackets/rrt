#include "gtest/gtest.h"
#include <Tree.hpp>
#include <2dplane/2dplane.hpp>

using namespace RRT;
using namespace Eigen;


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
