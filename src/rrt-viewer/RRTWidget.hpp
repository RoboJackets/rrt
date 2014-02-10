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
	void slot_clearObstacles();
	void slot_step();
	void slot_stepBig();

protected:
	void paintEvent(QPaintEvent *p);
	void drawTree(QPainter &painter,
		const RRT::Tree<Eigen::Vector2f> *rrt,
		const RRT::Node<Eigen::Vector2f> *solutionNode = NULL,
		QColor treeColor = Qt::blue,
		QColor solutionColor = Qt::red);

	void setupTree(RRT::Tree<Eigen::Vector2f> **treePP, Eigen::Vector2f source);
	QPointF pointFromNode(const RRT::Node<Eigen::Vector2f> *n);

	void step(int numTimes);

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
	RRT::Tree<Eigen::Vector2f> *_startTree;
	RRT::Tree<Eigen::Vector2f> *_goalTree;
	bool _draggingStart, _draggingGoal;

	//	track solution
	void resetSolution();
	RRT::Node<Eigen::Vector2f> *findBestPath(Eigen::Vector2f targetState, RRT::Tree<Eigen::Vector2f> *treeToSearch, int *depthOut);
	int _solutionLength;
	RRT::Node<Eigen::Vector2f> *_startSolutionNode, *_goalSolutionNode;

	///	the viewing area is divided up into rectangles
	///	@_blocked tracks whether or not they are obstacles.
	static const int GridWidth = 40, GridHeight = 30;
	bool _blocked[GridWidth][GridHeight];

	//	if you click down on an obstacle, you enter erase mode
	//	if you click down where there's no obstacle, you enter draw mode
	bool _editingObstacles, _erasingObstacles;

	///	if true, uses @_goalTree to grow from the goal towards @_startTree
	bool _bidirectional;
};
