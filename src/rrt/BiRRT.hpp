#include <rrt/Tree.hpp>
#include <limits.h>

namespace RRT {
/**
 * @brief Bi-directional RRT
 * @details It is often preferable to use two RRTs when searching the state
 * space with
 * one rooted at the source and one rooted at the goal.  When the two trees
 * intersect,
 * a solution has been found.
 */
template <typename T> class BiRRT {
public:
  BiRRT(std::shared_ptr<StateSpace<T>> stateSpace)
      : _startTree(stateSpace), _goalTree(stateSpace) {
    reset();
  }

  void reset() {
    _startTree.reset();
    _goalTree.reset();

    _iterationCount = 0;

    _startSolutionNode = nullptr;
    _goalSolutionNode = nullptr;
    _solutionLength = INT_MAX;
  }

  const Tree<T> &startTree() const { return _startTree; }
  const Tree<T> &goalTree() const { return _goalTree; }

  bool isASCEnabled() const { return _startTree.isASCEnabled(); }
  void setASCEnabled(bool checked) {
    _startTree.setASCEnabled(checked);
    _goalTree.setASCEnabled(checked);
  }

  float goalBias() const { return _startTree.goalBias(); }
  void setGoalBias(float goalBias) {
    _startTree.setGoalBias(goalBias);
    _goalTree.setGoalBias(goalBias);
  }

  int maxIterations() const { return _startTree.maxIterations(); }
  void setMaxIterations(int itr) {
    _startTree.setMaxIterations(itr);
    _goalTree.setMaxIterations(itr);
  }

  float waypointBias() const { return _startTree.waypointBias(); }
  void setWaypointBias(float waypointBias) {
    _startTree.setWaypointBias(waypointBias);
    _goalTree.setWaypointBias(waypointBias);
  }

  const std::vector<T> &waypoints() { return _startTree.waypoints(); }
  void setWaypoints(const std::vector<T> &waypoints) {
    _startTree.setWaypoints(waypoints);
    _goalTree.setWaypoints(waypoints);
  }

  float stepSize() const { return _startTree.stepSize(); }
  void setStepSize(float stepSize) {
    _startTree.setStepSize(stepSize);
    _goalTree.setStepSize(stepSize);
  }

  float maxStepSize() const { return _startTree.maxStepSize(); }
  void setMaxStepSize(float stepSize) {
    _startTree.setMaxStepSize(stepSize);
    _goalTree.setMaxStepSize(stepSize);
  }

  float goalMaxDist() const { return _startTree.goalMaxDist(); }
  void setGoalMaxDist(float maxDist) {
    _startTree.setGoalMaxDist(maxDist);
    _goalTree.setGoalMaxDist(maxDist);
  }

  /**
   * @brief Get the shortest path from the start to the goal
   *
   * @param vecOut The vector to place the solution in
   */
  void getPath(std::vector<T> &vecOut) {
    _startTree.getPath(vecOut, _startSolutionNode);
    _startTree.getPath(vecOut, _goalSolutionNode, true);
  }

  /**
   * @brief
   * @details Attempts to add a new node to each of the two trees.  If
   * a new solution is found that is shorter than any previous solution, we
   * store
   * it instead.
   */
  void grow() {
    int depth;
    Node<T> *otherNode;

    Node<T> *newStartNode = _startTree.grow();
    if (newStartNode) {
      otherNode = _findBestPath(newStartNode->state(), _goalTree, &depth);
      if (otherNode && depth + newStartNode->depth() < _solutionLength) {
        _startSolutionNode = newStartNode;
        _goalSolutionNode = otherNode;
        _solutionLength = newStartNode->depth() + depth;
      }
    }

    Node<T> *newGoalNode = _goalTree.grow();
    if (newGoalNode) {
      otherNode = _findBestPath(newGoalNode->state(), _startTree, &depth);
      if (otherNode && depth + newGoalNode->depth() < _solutionLength) {
        _startSolutionNode = otherNode;
        _goalSolutionNode = newGoalNode;
        _solutionLength = newGoalNode->depth() + depth;
      }
    }

    ++_iterationCount;
  }

  /**
   * @brief Grows the trees until we find a solution or run out of iterations.
   */
  void run() {
    for (int i = 0; i < _startTree.maxIterations(); i++) {
      grow();
      if (_startSolutionNode != nullptr)
        break;
    }
  }

  void setStartState(const T &start) {
    _startTree.setStartState(start);
    _goalTree.setGoalState(start);
  }
  const T &startState() const { return _startTree.startState(); }

  void setGoalState(const T &goal) {
    _startTree.setGoalState(goal);
    _goalTree.setStartState(goal);
  }
  const T &goalState() const { return _startTree.goalState(); }

  const Node<T> *startSolutionNode() { return _startSolutionNode; }

  const Node<T> *goalSolutionNode() { return _goalSolutionNode; }

  int iterationCount() const { return _iterationCount; }

protected:
  Node<T> *_findBestPath(const T &targetState, Tree<T> &treeToSearch,
                         int *depthOut) {
    Node<T> *bestNode = nullptr;
    int depth = INT_MAX;

    for (Node<T> *other : treeToSearch.allNodes()) {
      float dist =
          _startTree.stateSpace().distance(other->state(), targetState);
      if (dist < goalMaxDist() && other->depth() < depth) {
        bestNode = other;
        depth = other->depth();
      }
    }

    if (depthOut)
      *depthOut = depth;

    return bestNode;
  }

private:
  Tree<T> _startTree;
  Tree<T> _goalTree;

  int _iterationCount;

  int _solutionLength;
  Node<T> *_startSolutionNode, *_goalSolutionNode;
};
};
