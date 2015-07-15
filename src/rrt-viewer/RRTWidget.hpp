#pragma once

#include <QtWidgets>
#include <BiRRT.hpp>
#include <2dplane/AngleLimitedStateSpace.hpp>
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

  QPointF stateLocationToGui(const Eigen::Vector2f &stateLoc) const;
  Eigen::Vector2f guiToStateLocation(const QPointF &guiPt) const;
  float drawingScaleFactor() const;

  // If reverse=true, sets the out angle, otherwise sets the in angle
  template<bool reverse>
  AngleLimitedState calculateEndpointState(const Eigen::Vector2f &pos,
                                           Eigen::Vector2f &vel);
 private slots:
  void slot_run();
  void slot_runFast();
  void run_step();
  void slot_stop();
  void slot_reset();
  void slot_clearObstacles();
  void slot_step();
  void slot_stepBig();
  void slot_setGoalBias(int bias);      //  bias is from 0 to 100
  void slot_setWaypointBias(int bias);  //  bias is from 0 to 100
  void slot_setStepSize(double step);
  void slot_setCurvatureIncreaseFactor(
      double factor);  // value must be greater than 1

signals:
  void signal_stepped(int iterationCount);

 protected:
  void paintEvent(QPaintEvent *p);
  void drawTree(QPainter &painter, const RRT::Tree<AngleLimitedState> &rrt,
                const RRT::Node<AngleLimitedState> *solutionNode = NULL,
                QColor treeColor = Qt::blue, QColor solutionColor = Qt::red);
  void drawTerminalState(QPainter &painter, const Eigen::Vector2f &pos,
                         const Eigen::Vector2f &vel, const QColor &color);
  void drawObstacleGrid(QPainter &painter, const ObstacleGrid &obstacleGrid);

  QPointF pointFromNode(const RRT::Node<AngleLimitedState> *n);

  void printPath(const std::vector<AngleLimitedState> path);

  void step(int numTimes);

  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);

  bool mouseInGrabbingRange(QMouseEvent *event, const Eigen::Vector2f &pt);

 private:
  std::shared_ptr<AngleLimitedStateSpace> _stateSpace;
  RRT::BiRRT<AngleLimitedState> *_biRRT;

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

  QTimer *_runTimer;
};
