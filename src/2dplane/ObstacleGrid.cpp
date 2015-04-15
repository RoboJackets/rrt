#include "ObstacleGrid.hpp"
#include <stdlib.h>
#include <iostream>

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

float ObstacleGrid::nearestObstacle(const Vector2f &state) const {
    float initial = 2;
    float c = initial; // closest distance
    int x = (int)(state.x() / (_width / _discretizedWidth));
    int y = (int)(state.y() / (_height / _discretizedHeight));
    int dlim = 0;
    int ulim = 0;
    int llim = 0;
    int rlim = 0;
    
    for (int i = x - c; i < x + c && i >= 0 && i < discretizedWidth(); i++) {
        for (int j = y - c; j < y + c && j >= 0 && j < discretizedHeight(); j++) {
            bool obs = obstacleAt(i, j);
            //cout << obs << endl;
            if (obs) {
                float dist = sqrt((x-i)*(x-i)+(y-j)*(y-j));
                if (dist < c) {
                    c = dist;
                }
            } //else cout << "clear!" << endl;
        }
    }

    return c; //we require a fraction not the actual value
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
