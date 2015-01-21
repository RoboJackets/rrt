#include "RoboCupRRTWidget.hpp"

using namespace RRT;
using namespace Eigen;


RoboCupRRTWidget::RoboCupRRTWidget() {
    _stateSpace = make_shared<RoboCupStateSpace>(
        rect().width(),
        rect().height(),
        40,
        30,
        3.0,    //  max speed
        2.0     //  max accel
    );

    _biRRT = new BiRRT<RoboCupRobotState>(_stateSpace);

    _biRRT->setStartState(RoboCupRobotState(50, 50, 0, 1));
    RoboCupRobotState goal;
    goal.vel = Vector2f(1, 0);
    goal.pos = Vector2f(width() / 2.0, height() / 2.0);
    _biRRT->setGoalState(goal);
    _biRRT->setStepSize(0.02);
    _biRRT->setGoalMaxDist(20);

    setMouseTracking(true);
}

bool RoboCupRRTWidget::hasSolution() const {
    return _biRRT->startSolutionNode() != nullptr;
}

void RoboCupRRTWidget::slot_reset() {
    //  TODO: store waypoints or clear old solution

    _biRRT->reset();

    emit signal_stepped(0);
    update();
}

void RoboCupRRTWidget::slot_clearObstacles() {
    _stateSpace->obstacleGrid().clear();
    update();
}

void RoboCupRRTWidget::slot_setGoalBias(int bias) {
    _biRRT->setGoalBias((float)bias / 100.0f);
}

void RoboCupRRTWidget::slot_setWaypointBias(int bias) {
    _biRRT->setWaypointBias((float)bias / 100.0f);
}

void RoboCupRRTWidget::slot_setStepSize(double step) {
    _biRRT->setStepSize(step);
}

void RoboCupRRTWidget::step(int numTimes) {
    for (int i = 0; i < numTimes; i++) {
        _biRRT->grow();
    }

    //  TODO: store smoothed version of solution as _previousSolution

    emit signal_stepped(_biRRT->iterationCount());
    update();
}

void RoboCupRRTWidget::paintEvent(QPaintEvent *p) {
    RRTWidget::paintEvent(p);

    QPainter painter(this);

    drawObstacles(painter, _stateSpace->obstacleGrid());

    //  TODO: draw previous solution
    //  TODO: draw waypoint cache


    QColor startTreeColor = Qt::red;
    QColor goalTreeColor = Qt::darkGreen;
    QColor solutionColor = Qt::red;

    //  draw each tree (with solution if any)
    drawTree(painter, _biRRT->startTree(), _biRRT->startSolutionNode(), startTreeColor, solutionColor);
    drawTree(painter, _biRRT->goalTree(), _biRRT->goalSolutionNode(), goalTreeColor, solutionColor);

    //  draw goal and start states
    drawRobotState(painter, startTreeColor, 2, _biRRT->startState());
    drawRobotState(painter, goalTreeColor, 2, _biRRT->goalState());
}

void RoboCupRRTWidget::drawRobotState(QPainter &painter,
    const QColor &color,
    int sizeMultiplier, 
    const RoboCupRobotState &state)
{
    painter.setPen(QPen(color, 3*sizeMultiplier));

    //  draw position with a dot
    QPointF base(state.pos.x(), state.pos.y());
    painter.drawEllipse(base, 1*sizeMultiplier, 1*sizeMultiplier);

    Vector2f tipVec = 20*state.vel;  //  scale tipVec up so velocity is visible

    //  thinner pen for arrow body
    painter.setPen(QPen(color, 1*sizeMultiplier));

    //  draw main shaft of arrow
    Vector2f tipLoc = state.pos + tipVec;
    QPointF tip(tipLoc.x(), tipLoc.y());
    painter.drawLine(base, tip);

    //  TODO: draw arrow head
}

void RoboCupRRTWidget::drawTransition(QPainter &painter,
    const QColor &color,
    const RoboCupRobotState &from,
    const RoboCupRobotState &to)
{
    //  FIXME: this should be a curve, not a line
    painter.setPen(QPen(color, 1));
    QPointF fromPt(from.pos.x(), from.pos.y());
    QPointF toPt(to.pos.x(), to.pos.y());
    painter.drawLine(fromPt, toPt);
}

void RoboCupRRTWidget::drawTree(QPainter &painter,
    const Tree<RoboCupRobotState> &rrt,
    const Node<RoboCupRobotState> *solutionNode,
    const QColor &treeColor,
    const QColor &solutionColor)
{
    //  draw all nodes and transitions
    for (const Node<RoboCupRobotState> *node : rrt.allNodes()) {
        drawRobotState(
            painter,
            treeColor,
            1,
            node->state()
        );

        if (node->parent()) {
            drawTransition(
                painter,
                treeColor,
                node->state(),
                node->parent()->state()
            );
        }
    }

    if (solutionNode) {
        //  TODO: draw solution
    }
}
