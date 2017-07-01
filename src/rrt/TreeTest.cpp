#include <gtest/gtest.h>
#include <memory>
#include <rrt/2dplane/2dplane.hpp>
#include <rrt/2dplane/GridStateSpace.hpp>
#include <rrt/Tree.hpp>
#include <vector>

using namespace Eigen;
using namespace std;

namespace RRT {

TEST(Tree, Example_2dplane) {
    shared_ptr<Tree<Vector2d>> tree =
        TreeFor2dPlane(make_shared<GridStateSpace>(50, 50, 50, 50),
                       Vector2d(40, 40),  //	goal point
                       5);                //	step size

    //	give it plenty of iterations so it's not likely to fail
    const int maxIterations = 10000;
    tree.get()->setMaxIterations(maxIterations);
    tree.get()->setGoalMaxDist(5);

    tree.get()->setStartState(Vector2d(10, 10));
    bool success = tree.get()->run();  //	run with the given starting point
    ASSERT_TRUE(success);
}

TEST(Tree, FailOnImpossibleRequest) {
    shared_ptr<Tree<Vector2d>> tree = TreeFor2dPlane(
        make_shared<GridStateSpace>(50, 50, 50, 50),
        Vector2d(60, 60),  //	goal point outside the bounds of the state space
        5);                //	step size

    //	give it plenty of iterations so it's not likely to fail
    const int maxIterations = 2000;
    tree.get()->setMaxIterations(maxIterations);
    tree.get()->setGoalMaxDist(5);

    tree.get()->setStartState(Vector2d(10, 10));
    bool success = tree.get()->run();  //	run with the given starting point
    ASSERT_FALSE(success);  // the rrt search should fail because the goal isn't
                            // reachable
}

TEST(Tree, getPath) {
    Vector2d start = {10, 10}, goal = {40, 40};
    shared_ptr<Tree<Vector2d>> tree =
        TreeFor2dPlane(make_shared<GridStateSpace>(50, 50, 50, 50),
                       goal,  //	goal point
                       5);    //	step size

    //	give it plenty of iterations so it's not likely to fail
    const int maxIterations = 10000;
    tree.get()->setMaxIterations(maxIterations);
    tree.get()->setGoalMaxDist(5);

    tree.get()->setStartState(start);
    bool success = tree.get()->run();  //	run with the given starting point
    ASSERT_TRUE(success);

    //	get path in reverse order (end -> root)
    vector<Vector2d> path;
    tree.get()->getPath(&path, tree.get()->lastNode(), true);
    ASSERT_TRUE(path.size() > 1);
    EXPECT_EQ(start, path.back());

    //	get path in regular order (root -> end)
    path.clear();
    tree.get()->getPath(&path, tree.get()->lastNode(), false);
    ASSERT_TRUE(path.size() > 1);
    EXPECT_EQ(start, path.front());
}

TEST(Tree, ASC) {
    // test adaptive stepsize control
    shared_ptr<Tree<Vector2d>> tree =
        TreeFor2dPlane(make_shared<GridStateSpace>(50, 50, 50, 50),
                       Vector2d(40, 40),  //	goal point
                       5);                //	step size

    //	give it plenty of iterations so it's not likely to fail
    const int maxIterations = 10000;
    tree.get()->setMaxIterations(maxIterations);
    tree.get()->setGoalMaxDist(5);
    tree.get()->setMaxStepSize(10);
    tree.get()->setASCEnabled(true);

    tree.get()->setStartState(Vector2d(10, 10));
    bool success = tree.get()->run();  //	run with the given starting point
    ASSERT_TRUE(success);

    vector<Vector2d> path;
    tree.get()->getPath(&path, tree.get()->lastNode(), true);

    // Check to see if the nodes in the tree have uniform stepsize or varied.
    // Stepsizes should vary
    bool varied = false;
    for (int i = 1; !varied && i < path.size() - 2; i++) {
        Vector2d pathA = path[i] - path[i - 1];
        Vector2d pathB = path[i] - path[i + 1];
        double n = pathA.norm() / pathB.norm();
        if (n < 0.99 || n > 1.01) varied = true;
    }
    ASSERT_TRUE(varied);
}

}  // namespace RRT
