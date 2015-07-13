#include "gtest/gtest.h"
#include <BiRRT.hpp>
#include <2dplane/AngleLimitedStateSpace.hpp>
#include <cmath>

using namespace RRT;
using namespace std;
using namespace Eigen;


TEST(AngleLimitedStateSpace, distance) {
    AngleLimitedStateSpace env(100, 100, 20, 20);

    AngleLimitedState s1(Vector2f(10, 10), 0, true);
    AngleLimitedState s2(Vector2f(10, 20));
    AngleLimitedState s3(Vector2f(10, 80));

    //  because the angles are too far apart, their distance is in "tier 2"
    EXPECT_LT((s1.pos()-s2.pos()).norm(), env.distance(s1, s2));

    AngleLimitedState intermediate = env.intermediateState(s1, s3, 10);
    EXPECT_TRUE(env.transitionValid(s1, intermediate));

    intermediate = env.intermediateState(s1, s3, 10, true);
    // cout << "s1: " << s1 << endl;
    // cout << "s3: " << s3 << endl;
    // cout << "intermediate state: " << intermediate << endl;
    EXPECT_TRUE(env.transitionValid(intermediate, s1));
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
        AngleLimitedState(Vector2f(0, 0), M_PI/2, true),
        AngleLimitedState(Vector2f(0, 1), M_PI/2, true),
        AngleLimitedState(Vector2f(1, 1), 0, true)
    };
    AngleLimitedStateSpace ss(10, 10, 10, 10);

    for (auto& state : states) state.setMaxAngleDiff(M_PI/4-0.1);
    EXPECT_TRUE(ss.transitionValid(states[0], states[1]));
    EXPECT_FALSE(ss.transitionValid(states[1], states[2]));
    EXPECT_FALSE(ss.transitionValid(states[0], states[2]));

    for (auto& state : states) state.setMaxAngleDiff(M_PI);
    EXPECT_TRUE(ss.transitionValid(states[0], states[1]));
    EXPECT_TRUE(ss.transitionValid(states[1], states[2]));
    EXPECT_TRUE(ss.transitionValid(states[0], states[2]));

    for (auto& state : states) state.setMaxAngleDiff(M_PI/4 + 0.1);
}
