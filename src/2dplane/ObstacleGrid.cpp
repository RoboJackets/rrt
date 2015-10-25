#include "ObstacleGrid.hpp"
#include <stdlib.h>

using namespace Eigen;
using namespace std;


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

float ObstacleGrid::nearestObstacleDist(const Vector2f &state) const {
    float c = _maxDist; // closest distance
    //x and y are the indices of the cell that state is located in
    float x = (state.x() / (_width / _discretizedWidth));
    float y = (state.y() / (_height / _discretizedHeight));
    //here we loop through the cells around (x,y) to find the minimum distance of the point to the nearest obstacle
    for (int i = x - c; i < x + c && i >= 0 && i < discretizedWidth(); i++) {
        for (int j = y - c; j < y + c && j >= 0 && j < discretizedHeight(); j++) {
            bool obs = obstacleAt(i, j);
            if (obs) {
                float dist = sqrt((x-i)*(x-i)+(y-j)*(y-j));
                if (dist < c) {
                    c = dist;
                }
            }
        }
    }
    return c;
}

void ObstacleGrid::setMaxDist(float maxDist) {
    _maxDist = maxDist;
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
