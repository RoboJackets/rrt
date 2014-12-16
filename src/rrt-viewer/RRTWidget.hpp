#pragma once

#include <QtWidgets>
#include <Tree.hpp>
#include <Eigen/Dense>


/**
 * This widget creates an RRT tree for searching a 2d space and draws it.
 * It has methods (slots) for stepping and resetting tree growth.
 * You can also draw and erase obstacles for by clicking and dragging.
 */
class RRTWidget : public QWidget {
    Q_OBJECT

public:
    RRTWidget();

    /**
     * If true, the widget will grow a tree from the start and from the
     * goal point.  When the trees intersect, a solution has been found.
     */
    bool bidirectional() const;

private slots:
    void slot_reset();
    void slot_clearObstacles();
    void slot_step();
    void slot_stepBig();
    void slot_setBidirectional(int bidirectional);
    void slot_setGoalBias(int bias);    //  bias is from 0 to 100
    void slot_setStepSize(double step);

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

    void resetTrees();

    /**
     * The viewing area is divided up in to rectangles which can be blocked
     * or open, which is how we keep track of obstacles.  This method simply
     * converts a QPoint or Vector2f to integer coordinates represenging a
     * particular rectangle in the grid.
     */
    template<typename P>
    void getIntCoordsForPt(P pt, int &xOut, int &yOut) {
        xOut = pt.x() * GridWidth / rect().width();
        yOut = pt.y() * GridHeight / rect().height();
    }

private:
    RRT::Tree<Eigen::Vector2f> *_startTree;
    RRT::Tree<Eigen::Vector2f> *_goalTree;
    bool _draggingStart, _draggingGoal;

    //  track solution
    void resetSolution();
    RRT::Node<Eigen::Vector2f> *findBestPath(Eigen::Vector2f targetState, RRT::Tree<Eigen::Vector2f> *treeToSearch, int *depthOut);
    int _solutionLength;
    RRT::Node<Eigen::Vector2f> *_startSolutionNode, *_goalSolutionNode;

    /// the viewing area is divided up into rectangles
    /// @_blocked tracks whether or not they are obstacles.
    static const int GridWidth = 40, GridHeight = 30;
    bool _blocked[GridWidth][GridHeight];

    //  if you click down on an obstacle, you enter erase mode
    //  if you click down where there's no obstacle, you enter draw mode
    bool _editingObstacles, _erasingObstacles;

    /// if true, uses @_goalTree to grow from the goal towards @_startTree
    bool _bidirectional;

    float _goalBias;

    void updateStepSizes();
    float _stepSize;

    //  sets the goal state for both trees (note that the goal of @_goalTree is the start point)
    void updateTreeGoals();
};
