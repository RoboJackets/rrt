#include "gtest/gtest.h"
#include "Path.hpp"

#include <iostream>
using namespace std;

using namespace Planning;

TEST(Path, DownSampleVector) {
    vector<float> values = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    DownSampleVector(values, 3);
    EXPECT_EQ(3, values.size());

    vector<float> expected = {2, 5, 8};
    EXPECT_EQ(expected, values);
}

TEST(SmoothPath, LeavesEndpoints) {
    vector<float> path = {1, 2, 3};
    auto transitionValidator = [](const float& from, const float& to) {
        return true;
    };
    SmoothPath<float>(path, transitionValidator);

    // because all transitions are valid, the path should remove the middle
    // point, leaving, {1, 3}.
    vector<float> expected = {1, 3};
    EXPECT_EQ(expected, path);
}
