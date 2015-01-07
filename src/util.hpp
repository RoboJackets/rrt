#pragma once

#include <Eigen/Dense>


template<typename T>
bool inRange(T n, T min, T max) {
    return (n >= min) && (n <= max);
}
