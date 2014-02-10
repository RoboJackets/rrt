
#include "RRTWidget.hpp"
#include <2dplane/2dplane.hpp>
#include <stdio.h>

using namespace RRT;
using namespace Eigen;


RRTWidget::RRTWidget() {
	setFixedSize(600, 450);

	//	default to bidirectional
	_bidirectional = true;

	//	reset
	resetSolution();

	//	setup @_startTree
	_startTree = nullptr;
	setupTree(&_startTree, Vector2f(50, 50));

	//	setup @_goalTree
	_goalTree = nullptr;
	setupTree(&_goalTree, Vector2f(width() / 2.0, height() / 2.0));

	//	register for mouse events
	setMouseTracking(true);
	_draggingStart = false;
	_draggingGoal = false;
}

bool RRTWidget::bidirectional() const {
	return _bidirectional;
}

void RRTWidget::slot_reset() {
	resetTrees();
	update();
}

void RRTWidget::resetTrees() {
	//	begin fresh trees with the same starting points as before
	setupTree(&_startTree, _startTree->rootNode()->state());
	setupTree(&_goalTree, _goalTree->rootNode()->state());
}

void RRTWidget::slot_clearObstacles() {
	for (int x = 0; x < GridWidth; x++) {
		for (int y = 0; y < GridHeight; y++) {
			_blocked[x][y] = false;
		}
	}
	update();
}

void RRTWidget::slot_setBidirectional(int bidirectional) {
	if ((bool)bidirectional != _bidirectional) {
		_bidirectional = (bool)bidirectional;
		resetTrees();
		update();
	}
}

void RRTWidget::setupTree(Tree<Vector2f> **treePP, Vector2f start) {
	resetSolution();

	if (*treePP) delete *treePP;
	const float stepSize = 10;
	*treePP = TreeFor2dPlane(width(), height(), Vector2f(0,0), stepSize);

	//	note: the obstacle detection here isn't perfect, but it's good enough
	(*treePP)->transitionValidator = [&](const Vector2f &from, const Vector2f &to) {
		int x, y; getIntCoordsForPt<Vector2f>(from, x, y);
		if (_blocked[x][y]) return false;

		getIntCoordsForPt<Vector2f>(to, x, y);
		if (_blocked[x][y]) return false;

		return true;
	};

	(*treePP)->setup(start);
}

void RRTWidget::resetSolution() {
	_startSolutionNode = nullptr;
	_goalSolutionNode = nullptr;
	_solutionLength = INT_MAX;
}

void RRTWidget::slot_step() {
	step(1);
}

void RRTWidget::slot_stepBig() {
	step(100);
}

void RRTWidget::step(int numTimes) {
	for (int i = 0; i < numTimes; i++) {
		Node<Vector2f> *newNode = _startTree->grow();
		
		int depth;
		Node<Vector2f> *otherNode;

		if (newNode) {
			otherNode = findBestPath(newNode->state(), _goalTree, &depth);
			if (otherNode && depth + newNode->depth() < _solutionLength) {
				_startSolutionNode = newNode;
				_goalSolutionNode = otherNode;
				_solutionLength = newNode->depth() + depth;
			}
		}

		if (_bidirectional) {
			newNode = _goalTree->grow();

			if (newNode) {
				otherNode = findBestPath(newNode->state(), _startTree, &depth);
				if (otherNode && depth + newNode->depth() < _solutionLength) {
					_startSolutionNode = otherNode;
					_goalSolutionNode = newNode;
					_solutionLength = newNode->depth() + depth;
				}
			}
		}
	}
	update();
}

Node<Vector2f> *RRTWidget::findBestPath(Vector2f targetState, Tree<Vector2f> *treeToSearch, int *depthOut) {
	Node<Vector2f> *bestNode = nullptr;
	int depth = INT_MAX;

	const float maxDist = 12;

	for (Node<Vector2f> *other : treeToSearch->allNodes()) {
		Vector2f delta = other->state() - targetState;
		if (magnitude(delta) < maxDist && other->depth() < depth) {
			bestNode = other;
			depth = other->depth();
		}
	}

	if (depthOut) *depthOut = depth;

	return bestNode;
}

QPointF RRTWidget::pointFromNode(const Node<Vector2f> *n) {
	return QPointF(n->state().x(), n->state().y());
}

void RRTWidget::paintEvent(QPaintEvent *p) {
	QPainter painter(this);

	//	draw black border around widget
	painter.setPen(QPen (Qt::black, 3));
	painter.drawRect(rect());

	//	draw obstacles
	int rectW = rect().width() / GridWidth, rectH = rect().height() / GridHeight;
	painter.setPen(QPen(Qt::black, 2));
	for (int x = 0; x < GridWidth; x++) {
		for (int y = 0; y < GridHeight; y++) {
			if (_blocked[x][y]) {
				painter.fillRect(x * rectW, y * rectH, rectW, rectH, Qt::SolidPattern);
			}
		}
	}

	//	draw @_startTree
	drawTree(painter, _startTree, _startSolutionNode);

	//	draw @_goalTree
	if (_bidirectional) {
		drawTree(painter, _goalTree, _goalSolutionNode, Qt::darkGreen);
	}

	//	draw root as a red dot
	if (_startTree->rootNode()) {
		painter.setPen(QPen (Qt::red, 6));
		QPointF rootLoc = pointFromNode(_startTree->rootNode());
		painter.drawEllipse(rootLoc, 2, 2);
	}

	//	draw goal as a green dot
	if (_goalTree->rootNode()) {
		QPointF goalLoc = pointFromNode(_goalTree->rootNode());
		painter.setPen(QPen(Qt::green, 6));
		painter.drawEllipse(goalLoc, 2, 2);
	}
}

void RRTWidget::drawTree(QPainter &painter,
	const Tree<Vector2f> *rrt,
	const Node<Vector2f> *solutionNode,
	QColor treeColor,
	QColor solutionColor)
{
	//	node drawing radius
	const float r = 1;

	//	draw all the nodes and connections
	for (const Node<Vector2f> *node : rrt->allNodes()) {
		painter.setPen(QPen (treeColor, 1));
		QPointF loc = pointFromNode(node);
		painter.drawEllipse(loc, r, r);

		if (node->parent()) {
			//	draw edge
			painter.setPen(QPen(treeColor, 1));
			QPointF parentLoc = pointFromNode(node->parent());
			painter.drawLine(loc, parentLoc);
		}
	}

	//	draw solution
	if (solutionNode) {
        painter.setPen(QPen(solutionColor, 2));

		const Node<Vector2f> *node = solutionNode, *parent = solutionNode->parent();
		while (parent) {
			//	draw the edge
			QPointF from = pointFromNode(node);
			QPointF to = pointFromNode(parent);
			painter.drawLine(from, to);

			//	scooch
			node = parent;
			parent = parent->parent();
		}
	}
}


#pragma mark Mouse Events

bool RRTWidget::mouseInGrabbingRange(QMouseEvent *event, const Vector2f &pt) {
	float dx = event->pos().x() - pt.x();
	float dy = event->pos().y() - pt.y();
	return sqrtf( dx*dx + dy*dy ) < 15;
}

void RRTWidget::mousePressEvent(QMouseEvent *event) {
	if (mouseInGrabbingRange(event, _startTree->rootNode()->state())) {
		_draggingStart = true;
	} else if (mouseInGrabbingRange(event, _goalTree->rootNode()->state())) {
		_draggingGoal = true;
	} else {
		_editingObstacles = true;
		int x, y; getIntCoordsForPt<QPointF>(event->pos(), x, y);
		_erasingObstacles = _blocked[x][y];

		//	toggle the blocked state of clicked square
		_blocked[x][y] = !_erasingObstacles;
		update();
	}
}

void RRTWidget::mouseMoveEvent(QMouseEvent *event) {
	Vector2f point(event->pos().x(), event->pos().y());

	if (_draggingStart) {
		//	reset the tree with the new start pos
		setupTree(&_startTree, point);
		update();
	} else if (_draggingGoal) {
		//	set the new goal point
		setupTree(&_goalTree, point);
		update();
	} else if (_editingObstacles) {
		int x, y; getIntCoordsForPt<QPointF>(event->pos(), x, y);
		_blocked[x][y] = !_erasingObstacles;
		update();
	}
}

void RRTWidget::mouseReleaseEvent(QMouseEvent *event) {
	_draggingGoal = false;
	_draggingStart = false;
	_editingObstacles = false;
}
