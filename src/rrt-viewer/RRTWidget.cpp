
#include "RRTWidget.hpp"
#include <planning/Path.hpp>
#include <iostream>


using namespace RRT;
using namespace Eigen;
using namespace std;
using std::min;


const float AccelLimit = 2.0;   //  TODO: make this configurable in the gui
const float MaxMaxAngleDiff = M_PI / 6.0;

// TODO: move this function into AngleLimitedStateSpace
float maxAngleDiffForSpeed(float speed, float accelLimit, float stepSize, float maxValue) {
    //  Think of the segments of an rrt forming a polygon inscribed
    //  in a circle if each segment is at the max angle diff from the previous segment.
    //  The max acceleration allowed and current speed limit the curvature of our path,
    //  so we use these to determine the min radius of this circle.
    //  accelCentripetal = speed^2 / radius = accelLimit
    //  for inscribed polygons, sideLength = 2*radius*sin(pi/numSides)
    //  for regular polygons (equal side lengths), numSides*exteriorAngle = 2*pi
    //  the exterior angle is equal to the maxAngleDiff, which is what we're trying to find
    //  maxAngleDiff = 2*pi / numSides
    //  sideLength = RRT step size
    //  numSides = 2*pi/maxAngleDiff
    //  stepSize = 2*radius*sin(maxAngleDiff/2)
    //  radius = speed^2 / accelLimit
    //  stepSize = 2*speed^2/accelLimit * sin(maxAngleDiff/2)
    //  maxAngleDiff = 2*asin(stepSize*accelLimit/(2*speed^2))
    float maxAngleDiff = 2.0*asinf(stepSize*accelLimit/(2.0*powf(speed, 2.0)));
    return min(maxAngleDiff, maxValue);
}

RRTWidget::RRTWidget() {
    _stateSpace = make_shared<AngleLimitedStateSpace>(8.09,
                                        6.05,
                                        40,
                                        30);
    _biRRT = new BiRRT<AngleLimitedState>(_stateSpace);


    const float drawingScaleFactor = 100;
    setFixedSize(_stateSpace->width()*drawingScaleFactor,
        _stateSpace->height()*drawingScaleFactor);

    _waypointCacheMaxSize = 15;

    _startVel = Vector2f(1, 0);
    _goalVel = Vector2f(0, 2);

    //  setup birrt
    AngleLimitedState start(Vector2f(1, 1),
                            atan2f(_startVel.y(), _startVel.x()), true);
    start.setMaxAngleDiff(maxAngleDiffForSpeed(
        _startVel.norm(), AccelLimit, _biRRT->stepSize(), MaxMaxAngleDiff));
    _biRRT->setStartState(start);

    //  TODO: set curvature limits and decay rates for endpoint states

    AngleLimitedState goal(
        Vector2f(_stateSpace->width() / 2.0, _stateSpace->height() / 2.0),
        atan2f(_goalVel.y(), _goalVel.x()), true);
    goal.setMaxAngleDiff(maxAngleDiffForSpeed(
        _goalVel.norm(), AccelLimit, _biRRT->stepSize(), MaxMaxAngleDiff));
    goal.setReverse(true);
    _biRRT->setGoalState(goal);

    _biRRT->setStepSize(0.2);
    _biRRT->setGoalMaxDist(0.1);

    //  register for mouse events
    setMouseTracking(true);

    _draggingItem = DraggingNone;

    _runTimer = nullptr;
}

void RRTWidget::slot_reset() {
    //  store waypoint cache
    vector<AngleLimitedState> waypoints;
    if (_biRRT->startSolutionNode() && _biRRT->goalSolutionNode()) {

        vector<Vector2f> prevSol = _previousSolution;

        if (prevSol.size() > 0) {
            //  don't keep the start or end states
            prevSol.erase(prevSol.begin());
            prevSol.erase(prevSol.end());

            //  down-sample
            Planning::DownSampleVector<Vector2f>(prevSol, _waypointCacheMaxSize);
        }

        for (Vector2f &pos : prevSol) {
            waypoints.push_back(AngleLimitedState(pos));
        }
    } else {
        _previousSolution.clear();
    }

    _biRRT->reset();

    _biRRT->setWaypoints(waypoints);

    emit signal_stepped(0);

    update();
}

void RRTWidget::slot_clearObstacles() {
    _stateSpace->obstacleGrid().clear();

    update();
}

void RRTWidget::slot_setGoalBias(int bias) {
    _biRRT->setGoalBias((float)bias / 100.0f);
}

void RRTWidget::slot_setWaypointBias(int bias) {
    _biRRT->setWaypointBias((float)bias / 100.0f);
}

void RRTWidget::slot_step() {
    step(1);
}

void RRTWidget::slot_stepBig() {
    step(100);
}

void RRTWidget::slot_setStepSize(double step) {
    _biRRT->setStepSize(step);
}

void RRTWidget::slot_setMaxAngleDiffDecayFactor(double maxAngleDiffDecayFactor) {
    _stateSpace->setMaxAngleDiffDecay(maxAngleDiffDecayFactor);
}

void RRTWidget::slot_run() {
    if (!_runTimer) {
        _runTimer = new QTimer(this);
        connect(_runTimer, SIGNAL(timeout()), this, SLOT(run_step()));
        _runTimer->start(0);
    }
}

void RRTWidget::slot_runFast() {
    while (_biRRT->startSolutionNode() == nullptr) {
        step(20);
    }
    update();
}

void RRTWidget::slot_stop() {
    if (_runTimer) {
        delete _runTimer;
        _runTimer = nullptr;
    }
}

void RRTWidget::run_step() {
    if (_biRRT->startSolutionNode() == nullptr) {
        step(1);
    } else {
        delete _runTimer;
        _runTimer = nullptr;
    }
}

void RRTWidget::printPath(const vector<AngleLimitedState> path) {
    for (auto &state : path) {
        cout << state << endl;
    }
}

void RRTWidget::step(int numTimes) {
    for (int i = 0; i < numTimes; i++) {
        _biRRT->grow();
    }

    //  store solution
    _previousSolution.clear();
    if (_biRRT->startSolutionNode() != nullptr) {
        vector<AngleLimitedState> prevSolutionStates;
        _biRRT->getPath(prevSolutionStates);

        cout << "Raw path" << "------------------" << endl;
        printPath(prevSolutionStates);

        Planning::SmoothPath<AngleLimitedState>(
            prevSolutionStates, *_stateSpace,
            &AngleLimitedStateSpace::PathModifier);

        cout << endl << "Smoothed Path" << "-------------------" << endl;
        printPath(prevSolutionStates);

        for (auto &state : prevSolutionStates) _previousSolution.push_back(state.pos());
    }

    emit signal_stepped(_biRRT->iterationCount());

    update();
}

QPointF RRTWidget::pointFromNode(const Node<AngleLimitedState> *n) {
    return QPointF(n->state().pos().x(), n->state().pos().y());
}

QPointF vecToPoint(const Vector2f &vec) {
    return QPointF(vec.x(), vec.y());
}

void RRTWidget::drawObstacleGrid(QPainter &painter, const ObstacleGrid &obstacleGrid) {
    float rectW = obstacleGrid.width() / obstacleGrid.discretizedWidth();
    float rectH = obstacleGrid.height() / obstacleGrid.discretizedHeight();
    painter.setPen(QPen(Qt::black, 0.1));
    painter.setBrush(QBrush(Qt::black));
    for (int x = 0; x < obstacleGrid.discretizedWidth(); x++) {
        for (int y = 0; y < obstacleGrid.discretizedHeight(); y++) {
            if (obstacleGrid.obstacleAt(x, y)) {
                QRectF rect(x*rectW, y*rectH, rectW, rectH);
                painter.fillRect(rect, Qt::SolidPattern);
            }
        }
    }
}

void RRTWidget::paintEvent(QPaintEvent *p) {
    QPainter painter(this);

    float s = drawingScaleFactor();
    painter.scale(s, s);

    //  draw black border around widget
    painter.setPen(QPen(Qt::black, 0.02));
    QRectF bounds(0, 0, _stateSpace->width(), _stateSpace->height());
    bounds.adjust(0.02, 0.02, -0.02, -0.02);
    painter.drawRect(bounds);

    drawObstacleGrid(painter, _stateSpace->obstacleGrid());

    //  draw previous solution
    if (_previousSolution.size() > 0) {
        painter.setPen(QPen(Qt::yellow, 0.03));
        Vector2f prev;
        bool first = true;
        for (const Vector2f &curr : _previousSolution) {
            if (first) {
                first = false;
            } else {
                painter.drawLine(QPointF(prev.x(), prev.y()), QPointF(curr.x(), curr.y()));
            }
            prev = curr;
        }


        //  draw cubic bezier interpolation of waypoints
        painter.setPen(QPen(Qt::darkBlue, 0.05));
        painter.setBrush(Qt::NoBrush);
        QPainterPath path(vecToPoint(_previousSolution[0]));

        Vector2f prevControlDiff = -_startVel.normalized() * 0.5*min(_startVel.norm(), (_previousSolution[0] - _previousSolution[1]).norm());
        for (int i = 1; i < _previousSolution.size(); i++) {
            Vector2f waypoint = _previousSolution[i];
            Vector2f prevWaypoint = _previousSolution[i-1];

            Vector2f controlDir;
            float controlLength;
            if (i == _previousSolution.size() - 1) {
                controlLength = 0.5*min(_goalVel.norm(), (waypoint - prevWaypoint).norm());
                controlDir = -_goalVel.normalized();
            } else {
                //  using first derivative heuristic from Sprunk 2008 to determine the distance of the control point from the waypoint
                Vector2f nextWaypoint = _previousSolution[i+1];
                controlLength = 0.5*min( (waypoint - prevWaypoint).norm(), (nextWaypoint - waypoint).norm() );
                controlDir = ((prevWaypoint - waypoint).normalized() - (nextWaypoint - waypoint).normalized()).normalized();
            }
            Vector2f controlDiff = controlDir * controlLength;

            path.cubicTo(
                vecToPoint(prevWaypoint - prevControlDiff),
                vecToPoint(waypoint + controlDiff),
                vecToPoint(waypoint)
            );

            prevControlDiff = controlDiff;
        }

        painter.drawPath(path);
    }


    //  draw waypoint cache
    if (_biRRT->waypoints().size() > 0) {
        float r = 0.01;    //  radius to draw waypoint dots

        painter.setPen(QPen(Qt::lightGray, 0.1));
        for (const AngleLimitedState &waypoint : _biRRT->waypoints()) {
            painter.drawEllipse(QPointF(waypoint.pos().x(), waypoint.pos().y()), r, r);
        }
    }

    //  draw trees
    drawTree(painter, _biRRT->startTree(), _biRRT->startSolutionNode());
    drawTree(painter, _biRRT->goalTree(), _biRRT->goalSolutionNode(), Qt::darkGreen);

    //  draw start and goal states
    drawTerminalState(painter, _biRRT->startState().pos(), _startVel, Qt::red);
    drawTerminalState(painter, _biRRT->goalState().pos(), _goalVel, Qt::darkGreen);
}

void RRTWidget::drawTerminalState(QPainter &painter, const Vector2f &pos, const Vector2f &vel, const QColor &color) {
    //  draw point
    QColor seethroughColor = color;
    seethroughColor.setAlphaF(0.5);
    painter.setBrush(QBrush(seethroughColor));
    painter.setPen(QPen(seethroughColor, 0));
    QPointF rootLoc(pos.x(), pos.y());
    const float botDiameter = 0.18;
    painter.drawEllipse(rootLoc, botDiameter, botDiameter);


    Vector2f tipOffset = vel;
    Vector2f tipLocVec = pos + tipOffset;
    QPointF tipLoc(tipLocVec.x(), tipLocVec.y());

    //  draw arrow shaft
    painter.setPen(QPen(color, 0.03));
    painter.drawLine(rootLoc, tipLoc);

    //  draw arrow head
    Vector2f headBase = tipLocVec - tipOffset.normalized()*0.03;
    Vector2f perp = Vector2f(-tipOffset.y(), tipOffset.x()).normalized();
    Vector2f tipLeftVec = headBase + perp*0.03;
    Vector2f tipRightVec = headBase - perp*0.03;
    QPointF trianglePts[] = {
        tipLoc,
        QPointF(tipLeftVec.x(), tipLeftVec.y()),
        QPointF(tipRightVec.x(), tipRightVec.y())
    };
    painter.drawPolygon(trianglePts, 3);

    painter.save(); {
        painter.translate(rootLoc + QPointF(0.3, 0));
        painter.scale(1.0/drawingScaleFactor(), 1.0/drawingScaleFactor());
        painter.setPen(QPen(Qt::black, 0.01));
        painter.drawText(QPointF(), QString("%1 m/s").arg(vel.norm()));
    } painter.restore();
}

void RRTWidget::drawTree(QPainter &painter,
    const Tree<AngleLimitedState> &rrt,
    const Node<AngleLimitedState> *solutionNode,
    QColor treeColor,
    QColor solutionColor)
{
    //  node drawing radius
    const float r = 0.01;

    //  draw all the nodes and connections
    for (const Node<AngleLimitedState> *node : rrt.allNodes()) {
        painter.setPen(QPen (treeColor, 0.02));
        QPointF loc = pointFromNode(node);
        painter.drawEllipse(loc, r, r);

        if (node->parent()) {
            //  draw edge
            painter.setPen(QPen(treeColor, 0.01));
            QPointF parentLoc = pointFromNode(node->parent());
            painter.drawLine(loc, parentLoc);
        }
    }

    //  draw solution
    if (solutionNode) {
        painter.setPen(QPen(solutionColor, 0.01));

        const Node<AngleLimitedState> *node = solutionNode, *parent = solutionNode->parent();
        while (parent) {
            //  draw the edge
            QPointF from = pointFromNode(node);
            QPointF to = pointFromNode(parent);
            painter.drawLine(from, to);

            //  scooch
            node = parent;
            parent = parent->parent();
        }
    }
}


#pragma mark Mouse Events

bool RRTWidget::mouseInGrabbingRange(QMouseEvent *event, const Vector2f &pt) {
    Vector2f clickPos = guiToStateLocation(event->pos());

    float dx = clickPos.x() - pt.x();
    float dy = clickPos.y() - pt.y();
    return sqrtf( dx*dx + dy*dy ) < 0.1;
}

void RRTWidget::mousePressEvent(QMouseEvent *event) {
    if (mouseInGrabbingRange(event, _biRRT->startState().pos())) {
        _draggingItem = DraggingStart;
    } else if (mouseInGrabbingRange(event, _biRRT->goalState().pos())) {
        _draggingItem = DraggingGoal;
    } else if (mouseInGrabbingRange(event, _biRRT->startState().pos() + _startVel)) {
        _draggingItem = DraggingStartVel;
    } else if (mouseInGrabbingRange(event, _biRRT->goalState().pos() + _goalVel)) {
        _draggingItem = DraggingGoalVel;
    } else {
        _editingObstacles = true;
        Vector2f pos = guiToStateLocation(event->pos());
        Vector2i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(pos);
        _erasingObstacles = _stateSpace->obstacleGrid().obstacleAt(gridLoc);

        //  toggle the obstacle state of clicked square
        _stateSpace->obstacleGrid().obstacleAt(gridLoc) = !_erasingObstacles;
        update();
    }
}

void RRTWidget::mouseMoveEvent(QMouseEvent *event) {
    Vector2f point = guiToStateLocation(event->pos());

    const float minMatterableEndpointVel = 0.1;

    if (_draggingItem == DraggingStart) {
        //  reset the tree with the new start pos
        bool startVelMatters = _startVel.norm() > minMatterableEndpointVel;
        AngleLimitedState start(point, atan2f(_startVel.y(), _startVel.x()), startVelMatters);
        start.setMaxAngleDiff(maxAngleDiffForSpeed(_startVel.norm(), AccelLimit, _biRRT->stepSize(), MaxMaxAngleDiff));
        _biRRT->setStartState(start);
    } else if (_draggingItem == DraggingGoal) {
        //  set the new goal point
        bool goalVelMatters = _goalVel.norm() > minMatterableEndpointVel;
        AngleLimitedState goal(point, atan2f(_goalVel.y(), _goalVel.x()), goalVelMatters);
        goal.setMaxAngleDiff(maxAngleDiffForSpeed(_goalVel.norm(), AccelLimit, _biRRT->stepSize(), MaxMaxAngleDiff));
        goal.setReverse(true);
        _biRRT->setGoalState(goal);
    } else if (_draggingItem == DraggingStartVel) {
        _startVel = (point - _biRRT->startState().pos());
        bool startVelMatters = _startVel.norm() > minMatterableEndpointVel;
        AngleLimitedState start(_biRRT->startState().pos(), atan2f(_startVel.y(), _startVel.x()), startVelMatters);
        start.setMaxAngleDiff(maxAngleDiffForSpeed(_startVel.norm(), AccelLimit, _biRRT->stepSize(), MaxMaxAngleDiff));
        _biRRT->setStartState(start);
    } else if (_draggingItem == DraggingGoalVel) {
        _goalVel = (point - _biRRT->goalState().pos());
        bool goalVelMatters = _goalVel.norm() > minMatterableEndpointVel;
        AngleLimitedState goal(_biRRT->goalState().pos(), atan2f(_goalVel.y(), _goalVel.x()), goalVelMatters);
        goal.setMaxAngleDiff(maxAngleDiffForSpeed(_goalVel.norm(), AccelLimit, _biRRT->stepSize(), MaxMaxAngleDiff));
        goal.setReverse(true);
        _biRRT->setGoalState(goal);
    } else if (_editingObstacles) {
        Vector2i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(point);
        _stateSpace->obstacleGrid().obstacleAt(gridLoc) = !_erasingObstacles;
    }

    if (_draggingItem != DraggingNone || _editingObstacles) update();
}

QPointF RRTWidget::stateLocationToGui(const Vector2f &stateLoc) const {
    return QPointF(
        stateLoc.x() * drawingScaleFactor(),
        stateLoc.y() * drawingScaleFactor()
    );
}

float RRTWidget::drawingScaleFactor() const {
    return width() / _stateSpace->width();
}

Vector2f RRTWidget::guiToStateLocation(const QPointF &guiPt) const {
    return Vector2f(
        guiPt.x() / drawingScaleFactor(),
        guiPt.y() / drawingScaleFactor()
    );
}

void RRTWidget::mouseReleaseEvent(QMouseEvent *event) {
    _draggingItem = DraggingNone;
    _editingObstacles = false;
}
