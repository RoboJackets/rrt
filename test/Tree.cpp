#include "gtest/gtest.h"
#include "../Tree.hpp"

using namespace RRT;


TEST(Instantiation, Tree) {
	Tree<int> *tree = new Tree<int>();
}
