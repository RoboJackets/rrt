#include "ObstacleGrid.hpp"
#include <stdlib.h>

using namespace Eigen;


ObstacleGrid::ObstacleGrid(float width, float height, int discretizedWidth, int discretizedHeight) {
    _width = width;
    _height = height;
    _discretizedWidth = discretizedWidth;
    _discretizedHeight = discretizedHeight;
    
    _obstacles = (bool *)malloc(sizeof(bool) * discretizedWidth * discretizedHeight);

    clear();
}

ObstacleGrid::~ObstacleGrid() {
    free(_obstacles);
}

Vector2i ObstacleGrid::gridSquareForLocation(const Vector2f &loc) const {
    return Vector2i(loc.x() / width() * discretizedWidth(),
                    loc.y() / height() * discretizedHeight());
}

void ObstacleGrid::clear() {
    for (int x = 0; x < discretizedWidth(); x++) {
        for (int y = 0; y < discretizedHeight(); y++) {
            obstacleAt(x, y) = false;
        }
    }
}

bool &ObstacleGrid::obstacleAt(int x, int y) {
    return _obstacles[x + _discretizedWidth*y];
}

bool ObstacleGrid::obstacleAt(int x, int y) const {
    return _obstacles[x + _discretizedWidth*y];
}

bool &ObstacleGrid::obstacleAt(const Vector2i &gridLoc) {
    return obstacleAt(gridLoc.x(), gridLoc.y());
}

bool ObstacleGrid::obstacleAt(const Vector2i &gridLoc) const {
    return obstacleAt(gridLoc.x(), gridLoc.y());
}

int ObstacleGrid::discretizedWidth() const {
    return _discretizedWidth;
}

int ObstacleGrid::discretizedHeight() const {
    return _discretizedHeight;
}

float ObstacleGrid::width() const {
    return _width;
}

float ObstacleGrid::height() const {
    return _height;
}
