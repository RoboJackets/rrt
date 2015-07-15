#include "gtest/gtest.h"
#include <BiRRT.hpp>
#include <2dplane/AngleLimitedStateSpace.hpp>
#include <cmath>

using namespace RRT;
using namespace std;
using namespace Eigen;

TEST(AngleLimitedStateSpace, distance) {
  AngleLimitedStateSpace ss(100, 100, 100, 100);

  AngleLimitedState s1(Vector2f(10, 10), 0, true);
  AngleLimitedState s2(Vector2f(10, 20), 0, false);
  AngleLimitedState s3(Vector2f(10, 80), 0, false);

  //  because the angles are too far apart, their distance is in "tier 2"
  EXPECT_GT(ss.distance(s1, s2), (s1.pos() - s2.pos()).norm());

  AngleLimitedState intermediate = ss.intermediateState(s1, s3, 10);
  EXPECT_TRUE(ss.transitionValid(s1, intermediate));

  intermediate = ss.intermediateState(s1, s3, 10, false);
  EXPECT_TRUE(ss.transitionValid(s1, intermediate));
}

TEST(AngleLimitedState, intermediateState) {
  AngleLimitedStateSpace ss(100, 100, 100, 100);

  AngleLimitedState from(Vector2f(1, 1), 0, true);

  AngleLimitedState to(Vector2f(5, 1), 0, false);
  EXPECT_TRUE(ss.transitionValid(from, to));

  // @from and @to are on the same line, so @intermediate should be as well
  AngleLimitedState intermediate = ss.intermediateState(from, to, 1);
  EXPECT_FLOAT_EQ(1, (intermediate.pos() - from.pos()).norm());
  EXPECT_TRUE(ss.transitionValid(from, intermediate));
  EXPECT_FLOAT_EQ(0, intermediate.angle());

  // @to is greater than max angle diff away from @to's angle
  to.setPos(5 * Vector2f(cosf(from.maxAngleDiff() + 0.2),
                         sinf(from.maxAngleDiff() + 0.2)));
  intermediate = ss.intermediateState(from, to, 1);
  EXPECT_FLOAT_EQ(1, (intermediate.pos() - from.pos()).norm());
  EXPECT_TRUE(ss.transitionValid(from, intermediate));
  EXPECT_GE(from.maxAngleDiff(), intermediate.angle());

  // @to is greater than max angle diff away from @to's angle (but opposite
  // the direction in the above test)
  to.setPos(5 * Vector2f(cosf(-from.maxAngleDiff() - 0.2),
                         sinf(-from.maxAngleDiff() - 0.2)));
  intermediate = ss.intermediateState(from, to, 1);
  EXPECT_FLOAT_EQ(1, (intermediate.pos() - from.pos()).norm());
  EXPECT_TRUE(ss.transitionValid(from, intermediate));
  EXPECT_GE(from.maxAngleDiff(), -intermediate.angle());

  // reverse transitions
  from.setReverse(true);
  to.setPos(to.pos() + Vector2f(2, 1));
  EXPECT_FALSE(ss.transitionValid(from, to));
}

TEST(AngleLimitedStateSpace, reverse) {
  AngleLimitedStateSpace ss(100, 100, 100, 100);

  AngleLimitedState goal(Vector2f(1, 1), 0, true);
  goal.setReverse(true);

  AngleLimitedState r = ss.randomState();
  AngleLimitedState intermediate = ss.intermediateState(goal, r, 1, true);
  EXPECT_TRUE(ss.transitionValid(goal, intermediate));
}

TEST(AngleLimitedStateSpace, transitionValid) {
  // Initialize states:
  //
  // 1--->2
  // ^
  // |
  // 0
  //
  // State 0 is given a start angle of pi/2, the other two have angles
  // calculated as atan2(pos - prev.pos).
  vector<AngleLimitedState> states = {
      AngleLimitedState(Vector2f(0, 0), M_PI / 2),
      AngleLimitedState(Vector2f(0, 1), M_PI / 2),
      AngleLimitedState(Vector2f(1, 1), 0)};
  AngleLimitedStateSpace ss(10, 10, 10, 10);

  for (auto& state : states) state.setMaxAngleDiff(M_PI / 4 - 0.1);
  EXPECT_TRUE(ss.transitionValid(states[0], states[1]));
  EXPECT_FALSE(ss.transitionValid(states[1], states[2]));
  EXPECT_FALSE(ss.transitionValid(states[0], states[2]));

  for (auto& state : states) state.setMaxAngleDiff(M_PI);
  EXPECT_TRUE(ss.transitionValid(states[0], states[1]));
  EXPECT_TRUE(ss.transitionValid(states[1], states[2]));
  EXPECT_TRUE(ss.transitionValid(states[0], states[2]));
}

TEST(AngleLimitedStateSpace, transitionValidForwardToReverseTree) {
  AngleLimitedStateSpace ss(100, 100, 100, 100);

  // a test-case pulled from a crappy run in the rrt-viewer
  // it created a connection here and it shouldn't have, so it makes a good
  // test case
  AngleLimitedState ff(Vector2f(5.52986, 1.15213), -0.252489, true);
  ff.setMaxAngleDiff(0.523599);
  ff.setReverse(false);
  AngleLimitedState rr(Vector2f(5.53339, 1.14981), 2.92843, true);
  rr.setMaxAngleDiff(0.523599);
  rr.setReverse(true);
  cout << "ff: " << ff << endl;
  cout << "rr: " << rr << endl;
  cout << endl;
  EXPECT_FALSE(ss.transitionValid(ff, rr));
}
