#include <rrt/2dplane/PlaneStateSpace.hpp>

using namespace Eigen;

namespace RRT {

PlaneStateSpace::PlaneStateSpace(float width, float height) {
    _width = width;
    _height = height;
}

Vector2f PlaneStateSpace::randomState() const {
    return Vector2f(drand48() * width(), drand48() * height());
}

Vector2f PlaneStateSpace::intermediateState(const Vector2f &source, const Vector2f &target, float stepSize) const {
    Vector2f delta = target - source;
    delta = delta / delta.norm();   //  unit vector

    Vector2f val = source + delta * stepSize;
    return val;
}

double PlaneStateSpace::distance(const Eigen::Vector2f &from, const Eigen::Vector2f &to) const {
    Vector2f delta = from - to;
    return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
}

bool PlaneStateSpace::stateValid(const Vector2f &pt) const {
        return pt.x() >= 0
            && pt.y() >= 0
            && pt.x() < width()
            && pt.y() < height();
}

float PlaneStateSpace::width() const {
    return _width;
}

float PlaneStateSpace::height() const {
    return _height;
}

}  // namespace RRT
