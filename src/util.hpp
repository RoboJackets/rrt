#pragma once


template<typename T>
bool inRange(T n, T min, T max) {
    return (n >= min) && (n <= max);
}

//  constrains the angle to one between -pi and pi
float fixAngleRadians(float angle);
