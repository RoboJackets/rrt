#pragma once

#include <list>
#include <functional>


namespace RRT
{
	/**
	 * Base class for an rrt tree node
	 *
	 * The template parameter T is for storing the state that the Node node
	 * represents.
	 */
	template<typename T>
	class Node {
	public:
		Node(T &state, Node<T> *parent = NULL) {
			_parent = parent;
			_state = state;
			
			if (_parent) {
				_parent->_children.push_back(this);
			}
		}
		
		Node<T> *parent() const {
			return _parent;
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
		 * Node represents.  Generally this is a vector (could be 2d, 3d, etc)
		 */
		const T &state() const {
			return _state;
		}

	protected:
		T _state;
		std::list<Node<T> *> _children;
		Node<T> *_parent;
	};


	/**
	 * Base tree class for RRT trees.  This class provides the generic data
	 * structure for the tree and the nodes tha make it up and some general
	 * algorithms.  Because many parts of an RRT are implementation-/domain-
	 * specific, parts of it should be placed in callbacks (this is a TODO item
	 * for now).  Note: The callbacks used in this class are C++ lambdas.  A
	 * good tutorial on them can be found here:
	 * http://www.cprogramming.com/c++11/c++11-lambda-closures.html.
	 *
	 * An RRT tree searches a state space by randomly filling it in and
	 * connecting points to form a branching tree.  Once a branch of the tree
	 * reaches the goal and satisifes all of the constraints, a solution is
	 * found and returned.
	 *
	 * The template parameter T is to specify the type that represents a state
	 * within the state-space that the tree is searching.  This could be a
	 * Geometry2d::Point or something else, but will generally be some sort of
	 * vector.
	 */
	template<typename T, typename P = Node<T> >
	class Tree {
	public:
		Tree() {
			setMaxIterations(100);

			//	FIXME: set callback stuff to NULL
		}

		virtual ~Tree() {
			reset();
		}

		/**
		 * Removes all Nodes from the tree so it can be run() again.
		 */
		void reset() {
		    // Delete all _nodes
		    for (Node<T> *pt : _nodes) delete pt;
		    _nodes.clear();
		}

		/**
		 * Executes the RRT algorithm with the given start state.  The run()
		 * method calls reset() automatically before doing anything.
		 *
		 * @return a bool indicating whether or not it found a path to the goal
		 */
		bool run(const T &start) {
			reset();

			//	FIXME: assert that all callbacks are provided
			
			//	create root node from provided start state
			Node<T> *root = new Node<T>(start, NULL);
			_nodes.push_back(root);

			for (int i = 0; i < _maxIterations; i++) {
				//	pick a random state
				T randState = randomStateGenerator();

				//	attempt and add a new node to the tree in the direction of
				//	@randState
				Node<T> *newNode = extend(randState);
				if (newNode) {
					if (goalProximityChecker(newNode)) {
						return true;
					}
				}
			}

			//	we hit our iteration limit and didn't reach the goal :(
			return false;
		}

		/**
		 * The maximum number of random states in the state-space that we will
		 * try before giving up on finding a path to the goal.
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
		Node<T> *nearest(const T &state) {
			float bestDistance = -1;
		    Node<T> *best = NULL;
		    
		    for (Node<T> *other : _nodes) {
		    	float dist = distanceCalculator(other->state(), state);
		        if (bestDistance < 0 || dist < bestDistance) {
		            bestDistance = dist;
		            best = other;
		        }
		    }

		    return best;
		}

		/**
		 * Grow the tree in the direction of @state
		 *
		 * @return the new tree Node (may be NULL if we hit Obstacles)
		 * @param source The Node to connect from.  If source == NULL, then
		 *             the closest tree point is used
		 */
		virtual Node<T> *extend(const T &target, Node<T> *source = NULL) {
			//	if we weren't given a source point, try to find a close node
			if (!source) {
				source = nearest(target);
				if (!source) {
					return NULL;
				}
			}
			
			//	Get a state that's in the direction of @target from @source.
			//	This should take a step in that direction, but not go all the
			//	way unless the they're really close together.
			T intermediateState = intermediateStateGenerator(source, target);

			//	Make sure there's actually a direct path from @source to
			//	@intermediateState.  If not, abort
			if (!transitionValidator(source, intermediateState)) {
				return NULL;
			}

			// Add a node to the tree for this state
			Node<T> *n = new Node<T>(intermediateState, source);
			_nodes.push_back(n);
			return n;
		}

		/**
		 * Attempts to connect @state into the tree by repeatedly calling
		 * extend() to connect a series of new Nodes in series from the closest
		 * point already in the tree towards @state.
		 *
		 * @param state The state to try to connect into the tree
		 * @return true if the connection was successfully made, false otherwise
		 */
		virtual bool connect(const T &state, const unsigned int maxAttempts = 50) {			
			Node<T> *from = NULL;
			for (unsigned int i = 0; i < maxAttempts; ++i) {
				Node<T> *newNode = extend(state, from);
				
				//	there's not a direct path from @from to @state; abort
				if (!newNode) return false;
				
				//	we found a connection
				if (newNode->state() == state)
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
		 * @param callback The lambda to call for each state in the path
		 * @param reverse if true, the states will be sent from @dest to the
		 *                tree's root
		 */
		void getPath(std::function<void(const T &stateI)> callback, Node<T> *dest, const bool reverse = false) {
			Node<T> *node = dest;
			if (reverse) {
				while (node) {
					call(node->state());
					node = node->parent();
				}
			} else {
				//	order them correctly in a list
				std::list<Node<T> *> nodes;
				while (node) {
					nodes.pus_front(node);
					node = node->parent();
				}

				//	then pass them one-by-one to the callback
				for (Node<T> *n : nodes) callback(node->state());
			}
		}

		/**
		 * @return The first node or NULL if none
		 */
		Node<T> *rootNode() const {
			if (_nodes.empty()) return NULL;
			
			return _nodes.front();
		}

		/**
		 * @return The most recent Node added to the tree
		 */
		Node<T> *lastNode() const {
			if (_nodes.empty()) return NULL;
			
			return _nodes.back();
		}


		//
		//	Callbacks - These MUST be overridden before using the Tree
		//

		/**
		 * This callback determines if a given transition is valid.
		 */
		std::function<bool (const Node<T> *start, const T &newState)> transitionValidator;

		/**
		 * Override this to provide a way for the Tree to generate random states.
		 *
		 * @return a state that is randomly chosen from the state-space
		 */
		std::function<T (void)> randomStateGenerator;

		/**
		 * This callback accepts two states and returns the 'distance' between
		 * them.
		 */
		std::function<float (const T &stateA, const T &stateB)> distanceCalculator;

		/**
		 * Callback to see if a given Node is at or near enough to the goal.
		 * Note that the Tree never asks where the goal is, only if a given Node
		 * is near.
		 */
		std::function<bool (const Node<T> *node)> goalProximityChecker;

		/**
		 * Finds a state in the direction of @target from @source.state().
		 * This new state will potentially be added to the tree.  No need to do
		 * any validation on the state before returning, the tree will handle
		 * that.
		 */
		std::function<T (const Node<T> *source, const T &target)> intermediateStateGenerator;


	protected:
		/**
		 * A list of all Node objects in the tree.
		 */
		std::list<Node<T> *> _nodes;

		int _maxIterations;
	};
}
