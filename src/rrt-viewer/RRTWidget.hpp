#pragma once

#include <QtWidgets>
#include <Tree.hpp>
#include <Eigen/Dense>


/**
 * This widget creates an RRT tree for searching a 2d space and draws it.
 * It has methods (slots) for stepping and resetting tree growth.
 */
class RRTWidget : public QWidget {
	Q_OBJECT

public:
	RRTWidget();

private slots:
	void slot_reset();
	void slot_step();
	void slot_stepBig();

protected:
	void paintEvent(QPaintEvent *p);
	void setupTree(Eigen::Vector2f source = Eigen::Vector2f(50, 50));
	QPointF pointFromNode(const RRT::Node<Eigen::Vector2f> *n);

	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

	static bool mouseInGrabbingRange(QMouseEvent *event, const Eigen::Vector2f &pt);

	template<typename P>
	void getIntCoordsForPt(P pt, int &xOut, int &yOut) {
		xOut = pt.x() * GridWidth / rect().width();
		yOut = pt.y() * GridHeight / rect().height();
	}

private:
	RRT::Tree<Eigen::Vector2f> *_tree;
	Eigen::Vector2f _goalState;
	bool _draggingStart, _draggingGoal;

	///	the viewing area is divided up into rectangles
	///	@_blocked tracks whether or not they are obstacles.
	static const int GridWidth = 40, GridHeight = 30;
	bool _blocked[GridWidth][GridHeight];

	//	if you click down on an obstacle, you enter erase mode
	//	if you click down where there's no obstacle, you enter draw mode
	bool _editingObstacles, _erasingObstacles;
};
