#pragma once

namespace RRT {

/**
 * A state space represents the set of possible states for a planning problem.
 * This includes the obstacles that may be present and what state transitions
 * are valid.
 * This class is abstract and must be subclassed in order to provide actual
 * functionality.
 */
template <typename T>
class StateSpace {
public:
    StateSpace(){};
    virtual ~StateSpace(){};

    /**
     * Generate a random state within the bounds of the state space.
     *
     * @return A random state
     */
    virtual T randomState() const = 0;

    /**
     * Finds a state in the direction of @target from @source.state().
     * This new state will potentially be added to the tree.  No need to do
     * any validation on the state before returning, the tree will handle
     * that.
     */
    virtual T intermediateState(const T& source, const T& target,
                                float stepSize) const = 0;

    /**
     * An overloaded version designed for use in adaptive stepsize control.
     *
     * @param source The node in the tree to extend from
     * @param target The point in the space to extend to
     * @param minStepSize The minimum allowable stepsize the intermediate state
     * will be extended from source
     * @param maxStepSize The maximum allowable stepsize the intermediate state
     * will be extended from source
     *
     * @return A state in the direction of @target from @source.state()
     */
    virtual T intermediateState(const T& source, const T& target,
                                float minStepSize, float maxStepSize) const = 0;

    /**
     * @brief Calculate the distance between two states
     *
     * @param from Start state
     * @param to End state
     *
     * @return The distance between the states
     */
    virtual double distance(const T& from, const T& to) const = 0;

    /**
     * @brief Check if a state is within bounds and obstacle-free
     *
     * @param state The state to check
     * @return A boolean indicating validity
     */
    virtual bool stateValid(const T& state) const = 0;

    /**
     * @brief Check motion validity from one state to another
     * @details Returns a boolean indicating whether or not a direct motion from
     * one state to another is valid.
     *
     * @param from The start state
     * @param to The destination state
     *
     * @return A boolean indicating validity
     */
    virtual bool transitionValid(const T& from, const T& to) const = 0;

protected:
    float _minStepSize;
    float _maxStepSize;
};

}  // namespace RRT
