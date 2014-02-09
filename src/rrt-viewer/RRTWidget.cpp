
#include "RRTWidget.hpp"
#include <2dplane/2dplane.hpp>
#include <stdio.h>

using namespace RRT;
using namespace Eigen;


RRTWidget::RRTWidget() {
	setFixedSize(400, 300);

	_tree = NULL;
	setupTree();
	_goalState = Vector2f(width() / 2.0, height() / 2.0);

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
	_tree->goalProximityChecker = [&](const Vector2f &state) {
		Vector2f delta = state - this->_goalState;
		return magnitude(delta) < 12;
	};

	//	note: the obstacle detection here isn't perfect, but it's good enough
	_tree->transitionValidator = [&](const Vector2f &from, const Vector2f &to) {
		int x, y; getIntCoordsForPt<Vector2f>(from, x, y);
		if (_blocked[x][y]) return false;

		getIntCoordsForPt<Vector2f>(to, x, y);
		if (_blocked[x][y]) return false;

		return true;
	};

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

	//	get solution
	float solutionDist;
	const Node<Vector2f> *solutionNode = _tree->nearest(_goalState, &solutionDist);
	if (solutionNode && solutionDist > 12) solutionNode = nullptr;

	//	draw rrt tree
	drawTree(painter, _tree, solutionNode);

	//	draw root as a red dot
	if (_tree->rootNode()) {
		painter.setPen(QPen (Qt::red, 6));
		QPointF rootLoc = pointFromNode(_tree->rootNode());
		painter.drawEllipse(rootLoc, 2, 2);
	}

	//	draw goal as a green dot
	QPointF goalLoc = QPointF(_goalState.x(), _goalState.y());
	painter.setPen(QPen(Qt::green, 6));
	painter.drawEllipse(goalLoc, 2, 2);
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
			painter.setPen(QPen (Qt::blue, 1));
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
	if (mouseInGrabbingRange(event, _tree->rootNode()->state())) {
		_draggingStart = true;
	} else if (mouseInGrabbingRange(event, _goalState)) {
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
		setupTree(point);
		update();
	} else if (_draggingGoal) {
		//	set the new goal point
		_goalState = Vector2f(point);
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
