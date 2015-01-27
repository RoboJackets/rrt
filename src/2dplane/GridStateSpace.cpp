#include "GridStateSpace.hpp"
#include <util.hpp>
#include <stdexcept>
#include <math.h>

using namespace Eigen;
using namespace std;


GridStateSpace::GridStateSpace(float width, float height, int discretizedWidth, int discretizedHeight):
    PlaneStateSpace(width, height),
    _obstacleGrid(width, height, discretizedWidth, discretizedHeight) {
}

bool GridStateSpace::stateValid(const Vector2f &pt) const {
    return PlaneStateSpace::stateValid(pt) && !_obstacleGrid.obstacleAt(_obstacleGrid.gridSquareForLocation(pt));
}

bool GridStateSpace::transitionValid(const Vector2f &from, const Vector2f &to) const {
    return _obstacleGrid.transitionValid(from, to);
}

const ObstacleGrid &GridStateSpace::obstacleGrid() const {
    return _obstacleGrid;
}

ObstacleGrid &GridStateSpace::obstacleGrid() {
    return _obstacleGrid;
}
