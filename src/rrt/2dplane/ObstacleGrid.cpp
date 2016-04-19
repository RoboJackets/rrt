#include "ObstacleGrid.hpp"
#include <stdlib.h>
#include <iostream>

using namespace Eigen;
using namespace std;

namespace RRT {

ObstacleGrid::ObstacleGrid(float width, float height, int discretizedWidth,
                           int discretizedHeight) {
    _width = width;
    _height = height;
    _discretizedWidth = discretizedWidth;
    _discretizedHeight = discretizedHeight;

    _obstacles =
        (bool*)malloc(sizeof(bool) * discretizedWidth * discretizedHeight);

    clear();
}

ObstacleGrid::~ObstacleGrid() { free(_obstacles); }

Vector2i ObstacleGrid::gridSquareForLocation(const Vector2f& loc) const {
    return Vector2i(loc.x() / width() * discretizedWidth(),
                    loc.y() / height() * discretizedHeight());
}

float ObstacleGrid::nearestObstacleDist(const Vector2f& state,
                                        float maxDist) const {
    // x and y are the indices of the cell that state is located in
    float x = (state.x() / (_width / _discretizedWidth));
    float y = (state.y() / (_height / _discretizedHeight));
    int xSearchRad = maxDist * _discretizedWidth / _width;
    int ySearchRad = maxDist * _discretizedHeight / _height;
    // here we loop through the cells around (x,y) to find the minimum distance
    // of
    // the point to the nearest obstacle
    for (int i = x - xSearchRad; i <= x + xSearchRad; i++) {
        for (int j = y - ySearchRad; j <= y + ySearchRad; j++) {
            bool obs = obstacleAt(i, j);
            if (obs) {
                float xDist = (x - i) * _width / _discretizedWidth;
                float yDist = (y - j) * _height / _discretizedHeight;
                float dist = sqrtf(powf(xDist, 2) + powf(yDist, 2));
                if (dist < maxDist) {
                    maxDist = dist;
                }
            }
        }
    }

    // the boundaries of the grid count as obstacles
    maxDist = std::min(maxDist, state.x());             // left boundary
    maxDist = std::min(maxDist, width() - state.x());   // right boundary
    maxDist = std::min(maxDist, state.y());             // top boundary
    maxDist = std::min(maxDist, height() - state.y());  // bottom boundary

    return maxDist;
}

void ObstacleGrid::clear() {
    for (int x = 0; x < discretizedWidth(); x++) {
        for (int y = 0; y < discretizedHeight(); y++) {
            obstacleAt(x, y) = false;
        }
    }
}

bool& ObstacleGrid::obstacleAt(int x, int y) {
    return _obstacles[x + _discretizedWidth * y];
}

bool ObstacleGrid::obstacleAt(int x, int y) const {
    return _obstacles[x + _discretizedWidth * y];
}

bool& ObstacleGrid::obstacleAt(const Vector2i& gridLoc) {
    return obstacleAt(gridLoc.x(), gridLoc.y());
}

bool ObstacleGrid::obstacleAt(const Vector2i& gridLoc) const {
    return obstacleAt(gridLoc.x(), gridLoc.y());
}

int ObstacleGrid::discretizedWidth() const { return _discretizedWidth; }

int ObstacleGrid::discretizedHeight() const { return _discretizedHeight; }

float ObstacleGrid::width() const { return _width; }

float ObstacleGrid::height() const { return _height; }

}  // namespace RRT
