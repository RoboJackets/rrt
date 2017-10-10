#pragma once

#include <deque>
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include <functional>
#include <list>
#include <memory>
#include <rrt/StateSpace.hpp>
#include <stdexcept>
#include <stdlib.h>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace RRT {
/**
 * Base class for an RRT tree node.
 *
 * @param T The datatype representing the state in the space the RRT
 * will be searching.
 */
template <typename T>
class Node {
public:
    Node(const T& state, Node<T>* parent = nullptr, int dimensions = 2,
         std::function<void(T, double*)> TToArray = NULL)
        : _parent(parent), _state(state), _vec(dimensions) {
        if (_parent) {
            _parent->_children.push_back(this);
        }
        if (NULL == TToArray) {
            for (int i = 0; i < dimensions; i++) {
                _vec[i] = state[i];
            }
        } else {
            TToArray(state, _vec.data());
        }
    }

    const Node<T>* parent() const { return _parent; }

    /**
     * Gets the number of ancestors (parent, parent's parent, etc) that
     * the node has.
     * Returns 0 if it doesn't have a parent.
     */
    int depth() const {
        int n = 0;
        for (Node<T>* ancestor = _parent; ancestor != nullptr;
             ancestor = ancestor->_parent) {
            n++;
        }
        return n;
    }

    /**
     * The @state property is the point in the state-space that this
     * Node represents.  Generally this is a vector (could be 2d, 3d, etc)
     */
    const T& state() const { return _state; }

    std::vector<double>* coordinates() { return &_vec; }

private:
    std::vector<double> _vec;
    T _state;
    std::list<Node<T>*> _children;
    Node<T>* _parent;
};

/**
 * An RRT tree searches a state space by randomly filling it in and
 * connecting points to form a branching tree.  Once a branch of the tree
 * reaches the goal and satisifes all of the constraints, a solution is
 * found and returned.
 *
 * This provides a base class for RRT trees.  Because many parts of an RRT are
 * implementation-/domain-specific, several key functionalities are placed in
 * callbacks (C++ lambdas), which must be supplied by the user of this class.
 *
 * If adaptive stepsize control (ASC) is enabled, then the stepsize for
 * extending new nodes from the tree will be dynamically updated depending on
 * how close the nearest obstacle is. If there are no nearby obstacles, then the
 * stepsize will be extended in order to safely cover more ground. If there are
 * nearby obstacles, then the stepsize will shrink so that the RRT can take more
 * precise steps.
 *
 * USAGE:
 * 1) Create a new Tree with the appropriate StateSpace
 *    RRT::Tree<My2dPoint> tree(stateSpace, hashT, arrayToT, TToArray);
 *
 *    hashT is a function pointer to a hash function for T
 *    arrayToT is an optional function pointer to convert from an array of
 *doubles to T
 *    TToArray is an optional function pointer to convert from T to an array of
 *doubles
 *    arrayToT and TToArray must be provided if T does not possess that
 *functionality
 *
 * 2) Set the start and goal states
 *    tree->setStartState(s);
 *    tree->setGoalState(g);
 *
 * 3) (Optional) If adaptive stepsize control is enabled:
 *    _stateSpace->setMaxStepSize sets the maximum stepsize the tree can take
 * for any step.
 *
 * 4) Run the RRT algorithm!  This can be done in one of two ways:
 *    Option 1) Call the run() method - it will grow the tree
 *              until it finds a solution or runs out of iterations.
 *
 *    Option 2) Call grow() repeatedly
 *
 *    Either way works fine, just choose whatever works best for your
 *    application.
 *
 * 5) Use getPath() to get the series of states that make up the solution
 *
 * @param T The type that represents a state within the state-space that
 * the tree is searching.  This could be a 2D Point or something else,
 * but will generally be some sort of vector.
 */
template <typename T>
class Tree {
public:
    Tree(const Tree&) = delete;
    Tree& operator=(const Tree&) = delete;
    Tree(std::shared_ptr<StateSpace<T>> stateSpace,
         std::function<size_t(T)> hashT, int dimensions,
         std::function<T(double*)> arrayToT = NULL,
         std::function<void(T, double*)> TToArray = NULL)
        : _kdtree(flann::KDTreeSingleIndexParams()),
          _dimensions(dimensions),
          _nodemap(20, hashT) {
        _stateSpace = stateSpace;
        _arrayToT = arrayToT;
        _TToArray = TToArray;

        //  default values
        setStepSize(0.1);
        setMaxStepSize(5);
        setMaxIterations(1000);
        setASCEnabled(false);
        setGoalBias(0);
        setWaypointBias(0);
        setGoalMaxDist(0.1);
    }

    StateSpace<T>& stateSpace() { return *_stateSpace; }
    const StateSpace<T>& stateSpace() const { return *_stateSpace; }

    /**
     * The maximum number of random states in the state-space that we will
     * try before giving up on finding a path to the goal.
     */
    int maxIterations() const { return _maxIterations; }
    void setMaxIterations(int itr) { _maxIterations = itr; }

    /**
     * Whether or not the tree is to run with adaptive stepsize control.
     */
    bool isASCEnabled() const { return _isASCEnabled; }
    void setASCEnabled(bool checked) { _isASCEnabled = checked; }

    /**
     * @brief The chance we extend towards the goal rather than a random point.
     * @details At each iteration of the RRT algorithm, we extend() towards a
     *     particular state.  The goalBias is a number in the range [0, 1] that
     *     determines what proportion of the time we extend() towards the goal.
     *     The rest of the time, we extend() towards a random state.
     */
    double goalBias() const { return _goalBias; }
    void setGoalBias(double goalBias) {
        if (goalBias < 0 || goalBias > 1) {
            throw std::invalid_argument(
                "The goal bias must be a number between 0.0 and 1.0");
        }
        _goalBias = goalBias;
    }

    /**
     * @brief The chance that we extend towards a randomly-chosen point from the
     * @waypoints vector
     */
    double waypointBias() const { return _waypointBias; }
    void setWaypointBias(double waypointBias) {
        if (waypointBias < 0 || waypointBias > 1) {
            throw std::invalid_argument(
                "The waypoint bias must be a number between 0.0 and 1.0");
        }
        _waypointBias = waypointBias;
    }

    /**
     * The waypoints vector holds a series of states that were a part of a
     * previously-generated successful path.
     * Setting these here and setting @waypointBias > 0 will bias tree growth
     * towards these
     */
    const std::vector<T>& waypoints() const { return _waypoints; }
    void setWaypoints(const std::vector<T>& waypoints) {
        _waypoints = waypoints;
    }
    void clearWaypoints() { _waypoints.clear(); }

    double stepSize() const { return _stepSize; }
    void setStepSize(double stepSize) { _stepSize = stepSize; }

    /// Max step size used in ASC
    double maxStepSize() const { return _maxStepSize; }
    void setMaxStepSize(double maxStep) { _maxStepSize = maxStep; }

    /**
     * @brief How close we have to get to the goal in order to consider it
     *     reached.
     * @details The RRT will continue to run unti we're within @goalMaxDist of
     *     the goal state.
     * reached.
     * @details The RRT will continue to run unti we're within @goalMaxDist of
     * the
     * goal state.
     */
    double goalMaxDist() const { return _goalMaxDist; }
    void setGoalMaxDist(double maxDist) { _goalMaxDist = maxDist; }

    /**
     * Executes the RRT algorithm with the given start state.
     *
     * @return a bool indicating whether or not it found a path to the goal
     */
    bool run() {
        //  grow the tree until we find the goal or run out of iterations
        for (int i = 0; i < _maxIterations; i++) {
            Node<T>* newNode = grow();

            if (newNode &&
                _stateSpace->distance(newNode->state(), _goalState) <
                    _goalMaxDist)
                return true;
        }

        //  we hit our iteration limit and didn't reach the goal :(
        return false;
    }

    /**
     * Removes nodes from _nodes and _nodemap so it can be run() again.
     */
    void reset(bool eraseRoot = false) {
        _kdtree = flann::Index<flann::L2_Simple<double>>(
            flann::KDTreeSingleIndexParams());
        if (eraseRoot) {
            _nodes.clear();
            _nodemap.clear();
        } else if (_nodes.size() > 1) {
            T root = rootNode()->state();
            _nodemap.clear();
            _nodes.clear();
            _nodes.emplace_back(root, nullptr, _dimensions, _TToArray);
            _nodemap.insert(std::pair<T, Node<T>*>(root, &_nodes.back()));
            if (_TToArray) {
                std::vector<double> data(_dimensions);
                _TToArray(root, data.data());
                _kdtree.buildIndex(
                    flann::Matrix<double>(data.data(), 1, _dimensions));
            } else {
                _kdtree.buildIndex(flann::Matrix<double>(
                    (double*)&(rootNode()->state()), 1, _dimensions));
            }
        }
    }

    /**
     * Picks a random state and attempts to extend the tree towards it.
     * This is called at each iteration of the run() method.
     */
    Node<T>* grow() {
        //  extend towards goal, waypoint, or random state depending on the
        //  biases and a random number
        double r =
            rand() /
            (double)RAND_MAX;  //  r is between 0 and one since we normalize it
        if (r < goalBias()) {
            return extend(goalState());
        } else if (r < goalBias() + waypointBias() && _waypoints.size() > 0) {
            const T& waypoint = _waypoints[rand() % _waypoints.size()];
            return extend(waypoint);
        } else {
            return extend(_stateSpace->randomState());
        }
    }

    /**
     * Find the node int the tree closest to @state.  Pass in a double pointer
     * as the second argument to get the distance that the node is away from
     * @state. This method searches a k-d tree of the points to determine
     */
    Node<T>* nearest(const T& state, double* distanceOut = nullptr) {
        Node<T>* best = nullptr;

        // k-NN search (O(log(N)))
        flann::Matrix<double> query;
        if (NULL == _TToArray) {
            query = flann::Matrix<double>((double*)&state, 1,
                                          sizeof(state) / sizeof(0.0));
        } else {
            std::vector<double> data(_dimensions);
            _TToArray(state, data.data());
            query = flann::Matrix<double>(data.data(), 1,
                                          sizeof(state) / sizeof(0.0));
        }
        std::vector<int> i(query.rows);
        flann::Matrix<int> indices(i.data(), query.rows, 1);
        std::vector<double> d(query.rows);
        flann::Matrix<double> dists(d.data(), query.rows, 1);

        int n =
            _kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());

        if (distanceOut)
            *distanceOut = _stateSpace->distance(state, best->state());

        T point;
        if (NULL == _arrayToT) {
            point = (T)_kdtree.getPoint(indices[0][0]);
        } else {
            point = _arrayToT(_kdtree.getPoint(indices[0][0]));
        }

        return _nodemap[point];
    }

    /**
     * Grow the tree in the direction of @state
     *
     * @return the new tree Node (may be nullptr if we hit Obstacles)
     * @param target The point to extend the tree to
     * @param source The Node to connect from.  If source == nullptr, then
     *             the closest tree point is used
     */
    virtual Node<T>* extend(const T& target, Node<T>* source = nullptr) {
        //  if we weren't given a source point, try to find a close node
        if (!source) {
            source = nearest(target, nullptr);
            if (!source) {
                return nullptr;
            }
        }

        //  Get a state that's in the direction of @target from @source. This
        //  should take a step in that direction, but not go all the way unless
        //  the they're really close together.
        T intermediateState;
        if (_isASCEnabled) {
            intermediateState = _stateSpace->intermediateState(
                source->state(), target, stepSize(), maxStepSize());
        } else {
            intermediateState = _stateSpace->intermediateState(
                source->state(), target, stepSize());
        }

        //  Make sure there's actually a direct path from @source to
        //  @intermediateState.  If not, abort
        if (!_stateSpace->transitionValid(source->state(), intermediateState)) {
            return nullptr;
        }

        // Add a node to the tree for this state
        _nodes.emplace_back(intermediateState, source, _dimensions, _TToArray);
        _kdtree.addPoints(flann::Matrix<double>(
            _nodes.back().coordinates()->data(), 1, _dimensions));
        _nodemap.insert(
            std::pair<T, Node<T>*>(intermediateState, &_nodes.back()));
        return &_nodes.back();
    }

    /**
     * Get the path from the receiver's root point to the dest point
     *
     * @param callback The lambda to call for each state in the path
     * @param dest The node in the tree to get the path for. If nullptr, will
     *     use the the last point added to the @_nodes vector. If run() was just
     *     called successfully, this node will be the one last created that is
     *     closest to the goal.
     * @param reverse if true, the states will be sent from @dest to the tree's
     *     root
     */
    void getPath(std::function<void(const T& stateI)> callback,
                 const Node<T>* dest = nullptr, bool reverse = false) const {
        const Node<T>* node = (dest != nullptr) ? dest : lastNode();
        if (reverse) {
            while (node) {
                callback(node->state());
                node = node->parent();
            }
        } else {
            // collect states in list in leaf -> root order
            std::vector<const Node<T>*> nodes;
            while (node) {
                nodes.push_back(node);
                node = node->parent();
            }

            // pass them one-by-one to the callback, reversing the order so
            // that the callback is called with the start point first and the
            // dest point last
            for (auto itr = nodes.rbegin(); itr != nodes.rend(); itr++) {
                callback((*itr)->state());
            }
        }
    }

    /**
     * The same as the first getPath() method, but appends the states to a given
     * output vector rather than executing a callback.
     *
     * @param vectorOut The vector to append the states along the path
     */
    void getPath(std::vector<T>* vectorOut, const Node<T>* dest = nullptr,
                 bool reverse = false) const {
        getPath([&](const T& stateI) { vectorOut->push_back(stateI); }, dest,
                reverse);
    }

    /**
     * The same as the first getPath() method, but returns the vector of states
     * instead of executing a callback.
     */
    std::vector<T> getPath(const Node<T>* dest = nullptr,
                           bool reverse = false) const {
        std::vector<T> path;
        getPath(&path, dest, reverse);
        return path;
    }

    /**
     * @return The root node or nullptr if none exists
     */
    const Node<T>* rootNode() const {
        if (_nodes.empty()) return nullptr;

        return &_nodes.front();
    }

    /**
     * @return The most recent Node added to the tree
     */
    const Node<T>* lastNode() const {
        if (_nodes.empty()) return nullptr;

        return &_nodes.back();
    }

    /**
     * All the nodes
     */
    const std::deque<Node<T>>& allNodes() const { return _nodes; }

    /**
     * @brief The start state for this tree
     */
    const T& startState() const {
        if (_nodes.empty())
            throw std::logic_error("No start state specified for RRT::Tree");
        else
            return rootNode()->state();
    }
    void setStartState(const T& startState) {
        reset(true);

        //  create root node from provided start state
        _nodes.emplace_back(startState, nullptr, _dimensions, _TToArray);
        _nodemap.insert(std::pair<T, Node<T>*>(startState, &_nodes.back()));
        if (_TToArray) {
            std::vector<double> data(_dimensions);
            _TToArray(rootNode()->state(), data.data());
            _kdtree.buildIndex(
                flann::Matrix<double>(data.data(), 1, _dimensions));
        } else {
            _kdtree.buildIndex(flann::Matrix<double>(
                (double*)&(rootNode()->state()), 1, _dimensions));
        }
    }

    /**
     * @brief The goal this tree is trying to reach.
     */
    const T& goalState() const { return _goalState; }
    void setGoalState(const T& goalState) { _goalState = goalState; }

protected:
    /**
     * A list of all Node objects in the tree.
     */
    std::deque<Node<T>> _nodes{};

    std::unordered_map<T, Node<T>*, std::function<size_t(T)>> _nodemap;

    T _goalState;

    const int _dimensions;

    int _maxIterations;

    bool _isASCEnabled;

    double _goalBias;

    /// used for Extended RRTs where growth is biased towards waypoints from
    /// previously grown tree
    double _waypointBias;
    std::vector<T> _waypoints{};

    double _goalMaxDist;

    double _stepSize;
    double _maxStepSize;

    flann::Index<flann::L2_Simple<double>> _kdtree;

    std::function<T(double*)> _arrayToT;

    std::function<void(T, double*)> _TToArray;

    std::shared_ptr<StateSpace<T>> _stateSpace{};
};
}  // namespace RRT
