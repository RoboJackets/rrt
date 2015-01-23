#include "ObstacleGrid.hpp"
#include <stdlib.h>
#include <util.hpp>

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

bool ObstacleGrid::pointInBounds(const Vector2f &pt) const {
    return pt.x() >= 0
        && pt.y() >= 0
        && pt.x() < width()
        && pt.y() < height();
}

bool ObstacleGrid::transitionValid(const Vector2f &from, const Vector2f &to) const {
    Vector2f delta = to - from;

    //  get the corners of this segment in integer coordinates.  This limits our intersection test to only the boxes in that square
    Vector2i discreteFrom = gridSquareForLocation(from);
    Vector2i discreteTo = gridSquareForLocation(to);
    int x1 = discreteFrom.x(), y1 = discreteFrom.y();
    int x2 = discreteTo.x(), y2 = discreteTo.y();


    //  order ascending
    if (x1 > x2) swap<int>(x1, x2);
    if (y1 > y2) swap<int>(y1, y2);

    float gridSqWidth = width() / discretizedWidth();
    float gridSqHeight = height() / discretizedHeight();

    //  check all squares from (x1, y1) to (x2, y2)
    for (int x = x1; x <= x2; x++) {
        for (int y = y1; y <= y2; y++) {
            if (obstacleAt(x, y)) {
                //  there's an obstacle here, so check for intersection

                //  the corners of this obstacle square
                Vector2f ulCorner(x * gridSqWidth, y * gridSqHeight);
                Vector2f brCorner(ulCorner.x() + gridSqWidth, ulCorner.y() + gridSqHeight);

                if (delta.x() != 0) {
                    /**
                     * Find slope and y-intercept of the line passing through @from and @to.
                     * y1 = m*x1+b
                     * b = y1-m*x1
                     */
                    float slope = delta.y() / delta.x();
                    float b = to.y() - to.x()*slope;

                    /*
                     * First check intersection with the vertical segments of the box.  Use y=mx+b for the from-to line and plug in the x value for each wall
                     * If the corresponding y-value is within the y-bounds of the vertical segment, it's an intersection.
                     */
                    float yInt = slope*ulCorner.x() + b;
                    if (inRange<float>(yInt, ulCorner.y(), brCorner.y())) return false;
                    yInt = slope*brCorner.x() + b;
                    if (inRange<float>(yInt, ulCorner.y(), brCorner.y())) return false;

                    /*
                     * Check intersection with horizontal sides of box
                     * y = k;
                     * y = mx+b;
                     * mx+b = k;
                     * mx = k - b;
                     * (k - b) / m = x;  is x within the horizontal range of the box?
                     */
                    if (slope == 0) return false;
                    float xInt = (ulCorner.y() - b) / slope;
                    if (inRange<float>(xInt, ulCorner.x(), brCorner.x())) return false;
                    xInt = (brCorner.y() - b) / slope;
                    if (inRange<float>(xInt, ulCorner.x(), brCorner.x())) return false;
                } else {
                    //  vertical line - slope undefined

                    //  see if it's within the x-axis bounds of this obstacle box
                    if (inRange<float>(from.x(), ulCorner.x(), brCorner.x())) {
                        //  order by y-value
                        //  note: @lower has a smaller value of y, but will appear higher visually on the screen due to qt's coordinate layout
                        Vector2f lower(from);
                        Vector2f higher(to);
                        if (higher.y() < lower.y()) swap<Vector2f>(lower, higher);

                        //  check for intersection based on y-values
                        if (lower.y() < ulCorner.y() && higher.y() > ulCorner.y()) return false;
                        if (lower.y() < brCorner.y() && higher.y() > brCorner.y()) return false;
                    }
                }

            }
        }
    }

    return true;
}
