
#include <Tree.hpp>
#include <Eigen/Dense>
#include <functional>
#include <math.h>

using namespace Eigen;
using namespace RRT;


const float kHeight = 100, kWidth = 100;


typedef struct {
	Vector2f origin;
	float w, h;
} Rect;


bool pointInRect(const Rect &rect, const Vector2f &pt) {
	return pt.x() > rect.origin.x() && pt.x() < rect.origin.x() + rect.w
			&& pt.y() > rect.origin.y() && pt.y() < rect.origin.y() + rect.h;
}

float magnitude(Vector2f &vec) {
	return sqrtf( powf(vec.x(), 2) + powf(vec.y(), 2) );
}


int main(int argc, char **argv) {
	Tree<Vector2f> *rrt = new Tree<Vector2f>();

	//	returns true if @pt is within a 100x100 rectangle
	std::function<bool (Vector2f&)> ptInBounds = [](Vector2f &pt) {
		return pt.x() > 0 && pt.x() < kHeight
				&& pt.y() > 0 && pt.y() < kWidth;
	};

	std::function<bool (const Vector2f&)> ptHitsObstacles = [](const Vector2f &pt) {
		return pointInRect({Vector2f(20, 20), 50, 50}, pt);
	};

	//	pick a random point within the plane
	rrt->randomStateGenerator = []() {
		return Vector2f(drand48() * kWidth, drand48() * kHeight);
	};

	//	distance
	rrt->distanceCalculator = [](const Vector2f &stateA, const Vector2f &stateB) {
		Vector2f delta = stateB - stateA;
		return magnitude(delta);
	};

	rrt->transitionValidator = [&](const Vector2f &start, const Vector2f &newState) {
		return !ptHitsObstacles(start) && !ptHitsObstacles(newState);
	};

	rrt->intermediateStateGenerator = [](const Vector2f &source, const Vector2f &target) {
		Vector2f delta = target - source;
		delta = delta / magnitude(delta);	//	unit vector
		const float step = 4.0;

		return source + delta * step;
	};

	rrt->goalProximityChecker = [](const Vector2f &state) {
		Vector2f delta = state - Vector2f(90, 90);
		return magnitude(delta) < 5;
	};

	//	run rrt starting at (1,1)
	bool success = rrt->run(Vector2f(1, 1));

	printf("%s\n", success ? "SUCCESS!" : "FAILURE :("); 

	return 0;
}
