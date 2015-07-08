#include "util.hpp"
#include <cmath>

float fixAngleRadians(float angle) {
    //  normalize
    while (angle > M_PI) angle -= 2.0*M_PI;
    while (angle < -M_PI) angle += 2.0*M_PI;
    return angle;
}
