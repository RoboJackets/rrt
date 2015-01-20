#include "gtest/gtest.h"
#include <BiRRT.hpp>
#include <2dplane/GridStateSpace.hpp>

using namespace RRT;
using namespace std;


TEST(Instantiation, BiRRT) {
    BiRRT<Eigen::Vector2f> biRRT(make_shared<GridStateSpace>(50, 50, 50, 50));
}
