#pragma once

#include <Eigen/Dense>
#include <QQuickPaintedItem>
#include <QtQuick>
#include <QtWidgets>
#include <rrt/2dplane/GridStateSpace.hpp>
#include <rrt/BiRRT.hpp>

/**
 * This widget creates an RRT tree for searching a 2d space and draws it.
 * It has methods (slots) for stepping and resetting tree growth.
 * You can also draw and erase obstacles by clicking and dragging.
 *
 * This class is used within QML to show and interact with the rrt. The
 * Q_PROPERTY macros in here allow properties to be accessible within qml. The
 * slots declared below allow methods to be called within qml.
 */
class RRTWidget : public QQuickPaintedItem {
    Q_OBJECT

public:
    RRTWidget();

    Q_PROPERTY(int iterations READ iterations NOTIFY signal_stepped)
    int iterations() const { return _biRRT->iterationCount(); }

    Q_PROPERTY(double stepSize READ stepSize WRITE setStepSize)
    void setStepSize(double step);
    double stepSize() const { return _biRRT->stepSize(); }

    Q_PROPERTY(double goalBias READ goalBias WRITE setGoalBias)
    void setGoalBias(double bias);  //  bias is from 0 to 1
    double goalBias() const { return _biRRT->goalBias(); }

    Q_PROPERTY(double waypointBias READ waypointBias WRITE setWaypointBias)
    void setWaypointBias(double bias);  //  bias is from 0 to 1
    double waypointBias() const { return _biRRT->waypointBias(); }

    Q_PROPERTY(bool ascEnabled READ ascEnabled WRITE setASCEnabled)
    void setASCEnabled(bool enabled);
    double ascEnabled() const { return _biRRT->isASCEnabled(); }

public slots:
    void run();
    void stop();
    void reset();
    void clearObstacles();
    void step();
    void stepBig();

    // called on a timer interval by run()
    void _run_step();

signals:
    void signal_stepped();

protected:
    void paint(QPainter* p) override;
    void drawTree(QPainter& painter, const RRT::Tree<Eigen::Vector2d>& rrt,
                  const RRT::Node<Eigen::Vector2d>* solutionNode = NULL,
                  QColor treeColor = Qt::blue, QColor solutionColor = Qt::red);
    void drawTerminalState(QPainter& painter, const Eigen::Vector2d& pos,
                           const Eigen::Vector2d& vel, const QColor& color);

    QPointF pointFromNode(const RRT::Node<Eigen::Vector2d>* n);

    void _step(int numTimes);

    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);

    static bool mouseInGrabbingRange(QMouseEvent* event,
                                     const Eigen::Vector2d& pt);

private:
    std::shared_ptr<RRT::GridStateSpace> _stateSpace;
    RRT::BiRRT<Eigen::Vector2d>* _biRRT;

    Eigen::Vector2d _startVel, _goalVel;

    //  if you click down on an obstacle, you enter erase mode
    //  if you click down where there's no obstacle, you enter draw mode
    bool _editingObstacles, _erasingObstacles;

    enum {
        DraggingNone = 0,
        DraggingStart,
        DraggingGoal,
        DraggingStartVel,
        DraggingGoalVel
    } _draggingItem;

    int _waypointCacheMaxSize;

    std::vector<Eigen::Vector2d> _previousSolution;

    QTimer* _runTimer;
};
