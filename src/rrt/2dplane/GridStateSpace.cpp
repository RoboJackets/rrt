#include <math.h>
#include <rrt/2dplane/GridStateSpace.hpp>
#include <rrt/util.hpp>
#include <stdexcept>

using namespace Eigen;
using namespace std;

#include <iostream>

namespace RRT {

GridStateSpace::GridStateSpace(double width, double height, int discretizedWidth,
                               int discretizedHeight)
    : PlaneStateSpace(width, height),
      _obstacleGrid(width, height, discretizedWidth, discretizedHeight) {}

bool GridStateSpace::stateValid(const Vector2d& pt) const {
    return PlaneStateSpace::stateValid(pt) &&
           !_obstacleGrid.obstacleAt(_obstacleGrid.gridSquareForLocation(pt));
}

Vector2d GridStateSpace::intermediateState(const Vector2d& source,
                                           const Vector2d& target,
                                           double minStepSize,
                                           double maxStepSize) const {
    bool debug = false;

    Vector2d delta = target - source;
    delta = delta / delta.norm();  //  unit vector
    double dist = _obstacleGrid.nearestObstacleDist(source, maxStepSize * 2);

    double stepSize =
        (dist / maxStepSize) *
        minStepSize;  // scale based on how far we are from obstacles
    if (stepSize > maxStepSize) stepSize = maxStepSize;
    if (stepSize < minStepSize) stepSize = minStepSize;
    if (debug) {
        cout << "ASC intermediateState" << endl;
        cout << "  stepsize: " << minStepSize << endl;
        cout << "  nearest obs dist: " << dist << endl;
        cout << "  maximum stepsize: " << maxStepSize << endl;
        cout << "  new step: " << stepSize << endl;
    }

    Vector2d val = source + delta * stepSize;
    return val;
}

bool GridStateSpace::transitionValid(const Vector2d& from,
                                     const Vector2d& to) const {
    //  make sure we're within bounds
    if (!stateValid(to)) return false;

    Vector2d delta = to - from;

    //  get the corners of this segment in integer coordinates.  This limits our
    //  intersection test to only the boxes in that square
    Vector2i discreteFrom = _obstacleGrid.gridSquareForLocation(from);
    Vector2i discreteTo = _obstacleGrid.gridSquareForLocation(to);
    int x1 = discreteFrom.x(), y1 = discreteFrom.y();
    int x2 = discreteTo.x(), y2 = discreteTo.y();

    //  order ascending
    if (x1 > x2) swap<int>(x1, x2);
    if (y1 > y2) swap<int>(y1, y2);

    double gridSqWidth = width() / _obstacleGrid.discretizedWidth();
    double gridSqHeight = height() / _obstacleGrid.discretizedHeight();

    //  check all squares from (x1, y1) to (x2, y2)
    for (int x = x1; x <= x2; x++) {
        for (int y = y1; y <= y2; y++) {
            if (_obstacleGrid.obstacleAt(x, y)) {
                //  there's an obstacle here, so check for intersection

                //  the corners of this obstacle square
                Vector2d ulCorner(x * gridSqWidth, y * gridSqHeight);
                Vector2d brCorner(ulCorner.x() + gridSqWidth,
                                  ulCorner.y() + gridSqHeight);

                if (delta.x() != 0) {
                    /**
                     * Find slope and y-intercept of the line passing through
                     * @from and
                     * @to.
                     * y1 = m*x1+b
                     * b = y1-m*x1
                     */
                    double slope = delta.y() / delta.x();
                    double b = to.y() - to.x() * slope;

                    /*
                     * First check intersection with the vertical segments of
                     * the box.
                     * Use y=mx+b for the from-to line and plug in the x value
                     * for each
                     * wall
                     * If the corresponding y-value is within the y-bounds of
                     * the vertical
                     * segment, it's an intersection.
                     */
                    double yInt = slope * ulCorner.x() + b;
                    if (inRange<double>(yInt, ulCorner.y(), brCorner.y()))
                        return false;
                    yInt = slope * brCorner.x() + b;
                    if (inRange<double>(yInt, ulCorner.y(), brCorner.y()))
                        return false;

                    /*
                     * Check intersection with horizontal sides of box
                     * y = k;
                     * y = mx+b;
                     * mx+b = k;
                     * mx = k - b;
                     * (k - b) / m = x;  is x within the horizontal range of the
                     * box?
                     */
                    if (slope == 0) return false;
                    double xInt = (ulCorner.y() - b) / slope;
                    if (inRange<double>(xInt, ulCorner.x(), brCorner.x()))
                        return false;
                    xInt = (brCorner.y() - b) / slope;
                    if (inRange<double>(xInt, ulCorner.x(), brCorner.x()))
                        return false;
                } else {
                    //  vertical line - slope undefined

                    //  see if it's within the x-axis bounds of this obstacle
                    //  box
                    if (inRange<double>(from.x(), ulCorner.x(), brCorner.x())) {
                        //  order by y-value
                        //  note: @lower has a smaller value of y, but will
                        //  appear higher
                        //  visually on the screen due to qt's coordinate layout
                        Vector2d lower(from);
                        Vector2d higher(to);
                        if (higher.y() < lower.y())
                            swap<Vector2d>(lower, higher);

                        //  check for intersection based on y-values
                        if (lower.y() < ulCorner.y() &&
                            higher.y() > ulCorner.y())
                            return false;
                        if (lower.y() < brCorner.y() &&
                            higher.y() > brCorner.y())
                            return false;
                    }
                }
            }
        }
    }

    return true;
}

const ObstacleGrid& GridStateSpace::obstacleGrid() const {
    return _obstacleGrid;
}

ObstacleGrid& GridStateSpace::obstacleGrid() { return _obstacleGrid; }

}  // namespace RRT
