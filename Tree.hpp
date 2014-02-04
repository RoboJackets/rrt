#pragma once

#include <list>


namespace RRT
{
	/**
	 * Base class for an rrt tree node
	 *
	 * The template parameter T is for storing the state that the Node node represents.
	 * The type T should implement the '-' operator for measuring the distance between 2 states.
	 */
	template<typename T>
	class Node {
	public:
		Node(T &state, Node<T> *parent) : pos(p) {
			_parent = parent;
			_leaf = true;
			_state = state;
			
			if (_parent)
			{
				_parent->_children.push_back(this);
				_parent->_leaf = false;
			}
		}
		
		Node<T>* parent() const {
			return _parent;
		}
		
		bool isLeaf() const {
			return _leaf;
		}
		void setLeaf(bool isLeaf) {
			_leaf = isLeaf;
		}

		/**
		 * Gets the number of ancestors (parent, parent's parent, etc) that
		 * the node has.
		 * Returns 0 if it doesn't have a parent.
		 */
		int depth()  {
			int n = 0;
			for (Node<T> *ancestor = _parent; ancestor != NULL; ancestor = ancestor->_parent) {
				n++;
			}
			return n;
		}

		/**
		 * The @state property is the point in the state-space that this
		 * Node/node represents.  Generally this is a vector (could be 2d, 3d, etc)
		 */
		T &state() const {
			return _state;
		}

	protected:
		T _state;
		std::list<Node<T> *> _children;
		Node<T> *_parent;
		bool _leaf;
	};


	/**
	 * Base tree class for RRT trees.  This class provides the generic data structure
	 * for the tree and the nodes tha make it up and some general algorithms.  Because
	 * many parts of an RRT are implementation-/domain- specific, parts of it should be
	 * placed in callbacks (this is a TODO item for now).
	 * Note: The callbacks used in this class are C++ lambdas.  A good tutorial on them
	 * can be found here: http://www.cprogramming.com/c++11/c++11-lambda-closures.html.
	 *
	 * An RRT tree searches a state space by randomly filling it in and connecting
	 * points to form a branching tree.  Once a branch of the tree reaches the goal and
	 * satisifes all of the constraints, a solution is found and returned.
	 *
	 * The template parameter T is to specify the type that represents a state within
	 * the state-space that the tree is searching.  This could be a Geometry2d::Node or
	 * something else, but will generally be some sort of vector.
	 */
	template<typename T, typename P = Node<T> >
	class Tree {
	public:
		Tree() {
			setMaxIterations(100);
		}

		virtual ~Tree() {
			reset();
		}

		/**
		 * Removes all Nodes from the tree so it can be run() again.
		 */
		void reset() {
		    // Delete all points
		    for (Node<T> *pt : points) delete pt;
		    points.clear();
		}

		/**
		 * Executes the RRT algorithm with the given start state.  The run()
		 * method calls reset() automatically before doing anything.
		 *
		 * @return a bool indicating whether or not it found the goal
		 */
		bool run(const T &start) {
			reset();
			
			Node<T> *p = new Node(start, NULL);
			//	FIXME: throw exception if start isn't valid? - NO - see comment at top of header
			points.push_back(p);

			//	FIXME: run the algorithm here!
		}

		/**
		 * The maximum number of random points in the state-space that we will try
		 * before giving up on finding a path to the goal.
		 */
		int maxIterations() const {
			return _maxIterations;
		}
		void setMaxIterations(int itr) {
			_maxIterations = itr;
		}

		/**
		 * Find the point of the tree closest to @state
		 */
		Node<T> *nearest(T state) {
			float bestDistance = -1;
		    Node<T> *best = NULL;
		    
		    for (Node<T> *other : points)
		    {
		        float d = powf(other.state().distTo(state), 2);	//	magnitude squared
		        if (bestDistance < 0 || d < bestDistance)
		        {
		            bestDistance = d;
		            best = other;
		        }
		    }

		    return best;
		}

		/**
		 * Grow the tree in the direction of @pt
		 *
		 * @return the new tree Node (may be NULL if we hit Obstacles)
		 * @param base The Node to connect from.  If base == NULL, then
		 *             the closest tree point is used
		 */
		virtual Node<T> *extend(T state, Node<T> *base = NULL) {
			//	if we weren't given a base point, try to find a close point
			if (!base)
			{
				base = nearest(target);
				if (!base)
				{
					return NULL;
				}
			}
			
			T delta = target - base->target;
			float d = delta.mag();
			
			//	@intermediateState is the new point we will add to the tree
			//	we make sure its distance from @target is <= step
			T intermediateState;
			if (d < step) {
				intermediateState = target;
			} else {
				//	go in the direction of @target, but not as far
				intermediateState = base->state + delta / d * step;
			}
			
			//	abort if the segment isn't valid
			if (!segmentIsValid(base->state, target)) {
				return false;
			}
			
			// Add this point to the tree
			Node<T> *p = new Node<T>(intermediateState, base);
			points.push_back(p);
			return p;
		}

		/**
		 * Attempts to connect @state into the tree by repeatedly calling extend()
		 * to connect a series of new Nodes in series from the closest point already
		 * in the tree towards @state.
		 *
		 * @param state The state to try to connect into the tree
		 * @return true if the connection was successfully made, false otherwise
		 */
		virtual bool connect(const T &state) {
			//	try to reach the goal state
			const unsigned int maxAttemps = 50;
			
			Node<T> *from = NULL;
			for (unsigned int i = 0; i < maxAttemps; ++i)
			{
				Node<T> *newNode = extend(state, from);
				
				//	there's not a direct path from @from to @state; abort
				if (!newNode) return false;
				
				//	we found a connection
				if (newNode->state == state)
				{
					return true;
				}
				
				//	we found a waypoint to use, but we're not there yet
				from = newNode;
			}
			
			//	we used all of our attempts and didn't find a connection; abort
			return false;
		}

		/**
		 * Get the path from the receiver's root point to the dest point
		 *
		 * @param path the Path object to append the series of states to
		 * @param reverse if true, the points added to @path will be from dest to the tree's root
		 */
		//	FIXME: remove dependency on Path
		void getPath(Planning::Path &path, Node<T> *dest, const bool reverse = false) {
			//	build a list of Nodes between @dest and the receiver's root Node
			int n = 0;
			list<Node<T> *> points;
			while (dest)
			{
				if (rev) {
					points.push_back(dest);
				} else {
					points.push_front(dest);
				}
				dest = dest->parent();
				++n;
			}
			
			//	add the points in @points to the given Path
			path.points.reserve(path.points.size() + n);
			for (Node<T> *pt : points)
			{
				path.points.push_back(pt->state());
			}
		}

		/**
		 * @return The first point (the one passed to init()) or NULL if none
		 */
		Node<T> *rootNode() const {
			if (points.empty())
			{
				return NULL;
			}
			
			return points.front();
		}

		/**
		 * @return The most recent Node added to the tree
		 */
		Node<T> *lastNode() const {
			if (points.empty())
			{
				return NULL;
			}
			
			return points.back();
		}


		//
		//	Callbacks - These MUST be overridden before using the Tree
		//

		/**
		 * This callback determines if a given transition is valid.
		 */
		// bool [](Node<T> startPt, T &newState) transitionValidCallback;

		/**
		 * Override this to provide a way for the Tree to generate random states.
		 *
		 * @return a state that is randomly chosen from the state-space
		 */
		// T []() randomStateCallback;

		/**
		 * This callback accepts two states and returns the 'distance' between
		 * them.
		 */
		// float [](T &stateA, T &stateB) distanceCallback;

		/**
		 * Callback to see if a given Node is at or near enough to the goal.  Note
		 * that the Tree never asks where the goal is, only if a given Node is near
		 */
		// bool [](Node<T> *pt) pointNearGoalCallback;

		/**
		 * Finds a state in the direction of @target from @source.state().
		 * This new state will potentially be added to the tree.  No need to do any
		 * validation on the state before returning, the tree will handle that.
		 */
		// T [](Node<T> *source, T &target) inlineStateCallback;


	protected:
		/**
		 * A list of all Node objects in the tree.
		 */
		std::list<Node<T> *> points;

		int _maxIterations;
	};
}

//	FIXME: these are domain-specific callback items
// bool Tree::stateIsValid(T state) {
// 	return !_obstacles->hit(state);
// }

// bool Tree::segmentIsValid(T from, T to) {
// 	return !_obstacles->hit(Geometry2d::Segment(from, to));
// }
