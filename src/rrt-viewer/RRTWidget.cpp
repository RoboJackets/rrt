
#include "RRTWidget.hpp"
#include <2dplane/2dplane.hpp>
#include <stdio.h>

using namespace RRT;
using namespace Eigen;


RRTWidget::RRTWidget() {
	setFixedSize(400, 300);

	_tree = NULL;
	setupTree();
	Vector2f goal = Vector2f(width() / 2.0, height() / 2.0);
	setGoalState(goal);

	//	register for mouse events
	setMouseTracking(true);
	_draggingStart = false;
	_draggingGoal = false;
}

void RRTWidget::slot_reset() {
	//	begin a fresh tree with the same starting point as before
	setupTree(_tree->rootNode()->state());
	update();
}

void RRTWidget::setupTree(Vector2f start) {
	if (_tree) delete _tree;
	const float stepSize = 10;
	_tree = TreeFor2dPlane(width(), height(), _goalState, stepSize);
	setGoalState(_goalState);

	_tree->setup(start);
}

void RRTWidget::slot_step() {
	_tree->grow();
	update();
}

void RRTWidget::slot_stepBig() {
	for (int i = 0; i < 100; i++) {
		_tree->grow();
	}
	update();
}

void RRTWidget::setGoalState(const Vector2f &pt) {
	_goalState = pt;

	_tree->goalProximityChecker = [=](const Vector2f &state) {
		Vector2f delta = state - _goalState;
		return magnitude(delta) < 12;
	};

	update();
}

QPointF RRTWidget::pointFromNode(const Node<Vector2f> *n) {
	// float x = n->state().x() / (float)rect().width();
	// float y = n->state().y() / (float)rect().height();
	return QPointF(n->state().x(), n->state().y());
}

void RRTWidget::paintEvent(QPaintEvent *p) {
	QPainter painter(this);
	painter.setPen(QPen (Qt::black, 3));
	painter.drawRect(rect());

	const float r = 1;

	int closestDist = INT_MAX;
	const Node<Vector2f> *closeNode = NULL;

	//	draw all the nodes and connections
	for (const Node<Vector2f> *node : _tree->allNodes()) {
		painter.setPen(QPen (Qt::blue, 1));
		QPointF loc = pointFromNode(node);
		painter.drawEllipse(loc, r, r);

		if (node->parent()) {
			//	draw edge
			painter.setPen(QPen (Qt::blue, 1));
			QPointF parentLoc = pointFromNode(node->parent());
			painter.drawLine(loc, parentLoc);
		}

		//	see if this node has reached the goal
		if (_tree->goalProximityChecker(node->state()) && node->depth() < closestDist) {
			closeNode = node;
			closestDist = node->depth();
		}
	}

	//	draw root as a red dot
	if (_tree->rootNode()) {
		painter.setPen(QPen (Qt::red, 6));
		QPointF rootLoc = pointFromNode(_tree->rootNode());
		painter.drawEllipse(rootLoc, r*2, r*2);
	}

	//	draw goal as a green dot
	QPointF goalLoc = QPointF(_goalState.x(), _goalState.y());
	painter.setPen(QPen(Qt::green, 6));
	painter.drawEllipse(goalLoc, r*2, r*2);

	//	draw the solution in green
	if (closeNode) {
		painter.setPen(QPen (Qt::blue, 2));

		const Node<Vector2f> *node = closeNode, *parent = closeNode->parent();
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
	if (mouseInGrabbingRange(event, _tree->rootNode()->state())) {
		_draggingStart = true;
	} else if (mouseInGrabbingRange(event, _goalState)) {
		_draggingGoal = true;
	}
}

void RRTWidget::mouseMoveEvent(QMouseEvent *event) {
	Vector2f point(event->pos().x(), event->pos().y());

	if (_draggingStart) {
		//	reset the tree with the new start pos
		setupTree(point);
		update();
	} else if (_draggingGoal) {
		//	set the new goal point
		setGoalState(point);
		update();
	}
}

void RRTWidget::mouseReleaseEvent(QMouseEvent *event) {
	_draggingGoal = false;
	_draggingStart = false;
}
