#pragma once

#include <StateSpace.hpp>

#include <functional>
#include <iostream>
#include <vector>

// TODO: rename to Path
namespace Planning {

/**
 * @brief Reduce the vector to @maxSize length
 * @details We do this by sampling to evenly distribute deletions
 *
 * @param states The vector of T values to sample
 * @param maxSize Max length of the resulting vector
 */
template <typename T>
void DownSampleVector(std::vector<T> &states, size_t maxSize) {
    if (states.size() > maxSize) {
        int toDelete = states.size() - maxSize;
        float spacing = (float)states.size() / (float)toDelete;
        float i = 0.0;
        while (toDelete) {
            toDelete--;
            states.erase(states.begin() + (int)(i + 0.5));
            i += spacing - 1.0;
        }
    }
}

/// Default function used in SmoothPath below.  It simply erases points between
/// (but not including) the start and end indexes.
template<typename T>
inline void DefaultPathModifier(std::vector<T> &pts, int start, int end) {
    pts.erase(pts.begin() + start + 1, pts.begin() + end);
}

/// Removes unnecessary waypoints along the path.  If A->B->C is the path and
/// A->C is valid, we can cut out B.  The modifier function is responsible for
/// removing the intermediate states and fixing up the the start and end points
/// for the subpath if necessary.
/// @param modifier A function that should delete the points (but not including)
/// the start and end indexes and adjust the start and end points as needed.
// TODO: modifier should take in a pointer to @pts, not a reference
template <typename T>
void SmoothPath(
    std::vector<T> &pts,
    std::function<bool(const T &from, const T &to)> const &transitionValidator,
    std::function<void(std::vector<T> &pts, int start, int end)> const &
        modifier = &DefaultPathModifier<T>) {
    int span = 2;
    while (span + 1 <= pts.size()) {
        bool changed = false;
        for (int i = 0; i + span < pts.size(); i++) {
            if (transitionValidator(pts[i], pts[i + span])) {
                modifier(pts, i, i + span);
                changed = true;
            }
        }

        if (!changed) span++;
    }
}

/// @brief Same as the above SmoothPath function, but uses the StateSpace to
/// provide the transition validator.
/// TODO: make this templated by StateSpace and require that state spaces
/// provide their own path modifier?
template <typename T>
void SmoothPath(
    std::vector<T> &pts, const StateSpace<T> &stateSpace,
    std::function<void(std::vector<T> &pts, int start, int end)> const &
        modifier = &DefaultPathModifier<T>) {
    SmoothPath<T>(pts, [&stateSpace](const T &from, const T &to) {
        return stateSpace.transitionValid(from, to);
    }, modifier);
}
};
