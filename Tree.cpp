#include "Tree.hpp"

#include <stdio.h>
#include <iostream>

using namespace RRT;
using namespace std;


#pragma mark Node

Node::Node(const T &state, Node *parent) :
	pos(p)
{
	_parent = parent;
	_leaf = true;
	_state = state;
	
	if (_parent)
	{
		_parent->_children.push_back(this);
		_parent->_leaf = false;
	}
}

int Node::depth() {
	int n = 0;
	for (Node<T> *ancestor = _parent; ancestor != NULL; ancestor = ancestor->_parent) {
		n++;
	}
	return n;
}


#pragma mark Tree

Tree::Tree()
{
	setMaxIterations(100);
}

Tree::~Tree()
{
	reset();
}

void Tree::reset()
{
    // Delete all points
    for (Node<T> *pt : points) delete pt;
    points.clear();
}

void Tree::run(const T &start)
{
	reset();
	
	Node<T> *p = new Node(start, NULL);
	//	FIXME: throw exception if start isn't valid? - NO - see comment at top of header
	points.push_back(p);

	//	FIXME: run the algorithm here!
}

void Tree::getPath(Planning::Path<T> &path, Node<T> *dest, const bool rev)
{
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

Node<T> *Tree::nearest(T &state)
{
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

Node<T> *Tree::rootNode() const
{
	if (points.empty())
	{
		return NULL;
	}
	
	return points.front();
}

Node<T> *Tree::lastNode() const
{
	if (points.empty())
	{
		return NULL;
	}
	
	return points.back();
}


# pragma mark FixedStepTree

Node<T> *FixedStepTree::extend(T target, Node<T> *base)
{
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

bool Tree::connect(const T &state)
{
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
