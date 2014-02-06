
#include "2dplane.hpp"
#include <functional>
#include <math.h>

using namespace Eigen;
using namespace RRT;


bool pointInRect(const Rect &rect, const Vector2f &pt) {
	return pt.x() > rect.origin.x() && pt.x() < rect.origin.x() + rect.w
			&& pt.y() > rect.origin.y() && pt.y() < rect.origin.y() + rect.h;
}

float magnitude(Vector2f &vec) {
	return sqrtf( powf(vec.x(), 2) + powf(vec.y(), 2) );
}

Tree<Vector2f> *TreeFor2dPlane(float w, float h, Vector2f goal, float step) {
	Tree<Vector2f> *rrt = new Tree<Vector2f>();

	//	returns true if @pt is within a 100x100 rectangle
	std::function<bool (Vector2f&)> ptInBounds = [=](Vector2f &pt) {
		return pt.x() > 0 && pt.x() < h
				&& pt.y() > 0 && pt.y() < w;
	};

	std::function<bool (const Vector2f&)> ptHitsObstacles = [](const Vector2f &pt) {
		return pointInRect({Vector2f(20, 20), 50, 50}, pt);
	};

	//	pick a random point within the plane
	rrt->randomStateGenerator = [=]() {
		return Vector2f(drand48() * w, drand48() * h);
	};

	//	distance
	rrt->distanceCalculator = [](const Vector2f &stateA, const Vector2f &stateB) {
		Vector2f delta = stateB - stateA;
		return magnitude(delta);
	};

	rrt->transitionValidator = [=](const Vector2f &start, const Vector2f &newState) {
		return true;
		// return !ptHitsObstacles(start) && !ptHitsObstacles(newState);
	};

	rrt->intermediateStateGenerator = [=](const Vector2f &source, const Vector2f &target) {
		Vector2f delta = target - source;
		delta = delta / magnitude(delta);	//	unit vector

		Vector2f val = source + delta * step;
		return val;
	};

	rrt->goalProximityChecker = [=](const Vector2f &state) {
		Vector2f delta = state - goal;
		return magnitude(delta) < 5;
	};

	return rrt;
}

// int main(int argc, char **argv) {
// 	auto rrt = TreeFor2dPlane(100, 100, Vector2f(90, 90), 4);

// 	//	run rrt starting at (1,1)
// 	bool success = rrt->run(Vector2f(1, 1));

// 	printf("%s\n", success ? "SUCCESS!" : "FAILURE :("); 

// 	return 0;
// }
