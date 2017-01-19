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
 */
class RRTWidget : public QQuickPaintedItem {
    Q_OBJECT

public:
    RRTWidget();

    Q_PROPERTY(int iterations READ iterations NOTIFY signal_stepped)
    int iterations() const { return _biRRT->iterationCount(); }

    Q_PROPERTY(float stepSize READ stepSize WRITE setStepSize)
    void setStepSize(float step);
    float stepSize() const { return _biRRT->stepSize(); }

    Q_PROPERTY(float goalBias READ goalBias WRITE setGoalBias)
    void setGoalBias(float bias);  //  bias is from 0 to 1
    float goalBias() const { return _biRRT->goalBias(); }

    Q_PROPERTY(float waypointBias READ waypointBias WRITE setWaypointBias)
    void setWaypointBias(float bias);  //  bias is from 0 to 1
    float waypointBias() const { return _biRRT->waypointBias(); }

    Q_PROPERTY(bool ascEnabled READ ascEnabled WRITE setASCEnabled)
    void setASCEnabled(bool enabled);
    float ascEnabled() const { return _biRRT->isASCEnabled(); }

public slots:
    void run();
    void run_step();  // TODO: rename?
    void stop();
    void reset();
    void clearObstacles();
    void step();
    void stepBig();

signals:
    void signal_stepped();

protected:
    void paint(QPainter* p) override;
    void drawTree(QPainter& painter, const RRT::Tree<Eigen::Vector2f>& rrt,
                  const RRT::Node<Eigen::Vector2f>* solutionNode = NULL,
                  QColor treeColor = Qt::blue, QColor solutionColor = Qt::red);
    void drawTerminalState(QPainter& painter, const Eigen::Vector2f& pos,
                           const Eigen::Vector2f& vel, const QColor& color);

    QPointF pointFromNode(const RRT::Node<Eigen::Vector2f>* n);

    void _step(int numTimes);

    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);

    static bool mouseInGrabbingRange(QMouseEvent* event,
                                     const Eigen::Vector2f& pt);

private:
    std::shared_ptr<RRT::GridStateSpace> _stateSpace;
    RRT::BiRRT<Eigen::Vector2f>* _biRRT;

    Eigen::Vector2f _startVel, _goalVel;

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

    std::vector<Eigen::Vector2f> _previousSolution;

    QTimer* _runTimer;
};
