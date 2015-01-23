#include "gtest/gtest.h"
#include <BiRRT.hpp>
#include <2dplane/AngleLimitedStateSpace.hpp>
#include <math.h>

using namespace RRT;
using namespace std;
using namespace Eigen;


TEST(Distance, AngleLimitedStateSpace) {
    AngleLimitedStateSpace env(100, 100, 20, 20);

    AngleLimitedState s1(Vector2f(10, 10), 0, true);
    AngleLimitedState s2(Vector2f(10, 20));
    AngleLimitedState s3(Vector2f(10, 80));

    //  because the angles are too far apart, their distance should be in "tier 2"
    EXPECT_LT((s1.pos()-s2.pos()).norm(), env.distance(s1, s2));

    AngleLimitedState intermediate = env.intermediateState(s1, s3, 10);
    EXPECT_EQ(true, env.transitionValid(s1, intermediate));

    intermediate = env.intermediateState(s1, s3, 10, true);
    EXPECT_EQ(true, env.transitionValid(s1, intermediate, true));
}
