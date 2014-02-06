
#include <Tree.hpp>
#include <Eigen/Dense>


typedef struct {
	Eigen::Vector2f origin;
	float w, h;
} Rect;

bool pointInRect(const Rect &rect, const Eigen::Vector2f &pt);

float magnitude(Eigen::Vector2f &vec);

RRT::Tree<Eigen::Vector2f> *TreeFor2dPlane(
	float w,
	float h,
	Eigen::Vector2f goal,
	float step);
