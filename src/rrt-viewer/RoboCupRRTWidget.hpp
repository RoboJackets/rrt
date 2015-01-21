#pragma once

#include "RRTWidget.hpp"
#include <2dplane/RoboCupStateSpace.hpp>
#include <BiRRT.hpp>


class RoboCupRRTWidget : public RRTWidget {
    Q_OBJECT

public:
    RoboCupRRTWidget();

    void step(int numTimes);
    bool hasSolution() const;

    void paintEvent(QPaintEvent *p);
    void drawRobotState(QPainter &painter,
        const QColor &color,
        int sizeMultiplier,
        const RoboCupRobotState &state);
    void drawTransition(QPainter &painter,
        const QColor &color,
        const RoboCupRobotState &from,
        const RoboCupRobotState &to);
    void drawTree(QPainter &painter,
        const RRT::Tree<RoboCupRobotState> &rrt,
        const RRT::Node<RoboCupRobotState> *solutionNode,
        const QColor &treeColor,
        const QColor &solutionColor);


protected slots:
    void slot_reset();
    void slot_clearObstacles();
    void slot_setGoalBias(int bias);
    void slot_setWaypointBias(int bias);
    void slot_setStepSize(double step);


private:
    std::shared_ptr<RoboCupStateSpace> _stateSpace;
    RRT::BiRRT<RoboCupRobotState> *_biRRT;
};
