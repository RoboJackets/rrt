
#include "RRTWidget.hpp"
#include <rrt/2dplane/2dplane.hpp>
#include <rrt/planning/Path.hpp>

using namespace RRT;
using namespace Eigen;
using namespace std;

/// multiply velocity by this to get the length of the vector to draw
const float VelocityDrawingMultiplier = 12;

RRTWidget::RRTWidget() {
    Vector2f size(800, 600);
    _stateSpace = make_shared<GridStateSpace>(size.x(), size.y(), 40, 30);
    _biRRT = new BiRRT<Vector2f>(_stateSpace);

    _waypointCacheMaxSize = 15;

    //  setup birrt
    _biRRT->setStartState(size / 10);
    _biRRT->setGoalState(size / 2);
    _biRRT->setMaxStepSize(30);
    _biRRT->setGoalMaxDist(12);

    _startVel = Vector2f(1, 0);
    _goalVel = Vector2f(0, 1);

    //  register for mouse events
    setAcceptedMouseButtons(Qt::LeftButton);

    _draggingItem = DraggingNone;
    _editingObstacles = false;

    _runTimer = nullptr;
}

void RRTWidget::reset() {
    //  store waypoint cache
    vector<Vector2f> waypoints;
    if (_biRRT->startSolutionNode() && _biRRT->goalSolutionNode()) {
        waypoints = _previousSolution;
        if (waypoints.size() > 0) {
            //  don't keep the start or end states
            waypoints.erase(waypoints.begin());
            waypoints.erase(waypoints.end() - 1);

            //  down-sample
            RRT::DownSampleVector<Vector2f>(waypoints, _waypointCacheMaxSize);
        }
    } else {
        _previousSolution.clear();
    }

    _biRRT->reset();

    _biRRT->setWaypoints(waypoints);

    emit signal_stepped();

    update();
}

void RRTWidget::clearObstacles() {
    _stateSpace->obstacleGrid().clear();

    update();
}

void RRTWidget::setGoalBias(float bias) { _biRRT->setGoalBias(bias); }

void RRTWidget::setWaypointBias(float bias) { _biRRT->setWaypointBias(bias); }

void RRTWidget::setASCEnabled(bool enabled) { _biRRT->setASCEnabled(enabled); }

void RRTWidget::step() { _step(1); }

void RRTWidget::stepBig() { _step(100); }

void RRTWidget::setStepSize(float step) { _biRRT->setStepSize(step); }

void RRTWidget::run() {
    if (!_runTimer) {
        _runTimer = new QTimer(this);
        connect(_runTimer, SIGNAL(timeout()), this, SLOT(run_step()));
        _runTimer->start(0);
    }
}

void RRTWidget::stop() {
    if (_runTimer) {
        delete _runTimer;
        _runTimer = nullptr;
    }
}

void RRTWidget::run_step() {
    if (_biRRT->startSolutionNode() == nullptr) {
        _step(1);
    } else {
        delete _runTimer;
        _runTimer = nullptr;
    }
}

void RRTWidget::_step(int numTimes) {
    for (int i = 0; i < numTimes; i++) {
        _biRRT->grow();
    }

    //  store solution
    _previousSolution.clear();
    if (_biRRT->startSolutionNode() != nullptr) {
        _biRRT->getPath(_previousSolution);
        RRT::SmoothPath<Vector2f>(_previousSolution, *_stateSpace);
    }

    emit signal_stepped();

    update();
}

QPointF RRTWidget::pointFromNode(const Node<Vector2f>* n) {
    return QPointF(n->state().x(), n->state().y());
}

QPointF vecToPoint(const Vector2f& vec) { return QPointF(vec.x(), vec.y()); }

void RRTWidget::paint(QPainter* p) {
    QPainter& painter = *p;  // TODO: just use the pointer everywhere?

    //  draw black border around widget
    painter.setPen(QPen(Qt::black, 3));
    QRectF rect(0, 0, width(), height());
    painter.drawRect(rect);

    //  draw obstacles
    int rectW = width() / _stateSpace->obstacleGrid().discretizedWidth(),
        rectH = height() / _stateSpace->obstacleGrid().discretizedHeight();
    painter.setPen(QPen(Qt::black, 2));
    for (int x = 0; x < _stateSpace->obstacleGrid().discretizedWidth(); x++) {
        for (int y = 0; y < _stateSpace->obstacleGrid().discretizedHeight();
             y++) {
            if (_stateSpace->obstacleGrid().obstacleAt(x, y)) {
                painter.fillRect(x * rectW, y * rectH, rectW, rectH,
                                 Qt::SolidPattern);
            }
        }
    }

    //  draw previous solution
    if (_previousSolution.size() > 0) {
        painter.setPen(QPen(Qt::yellow, 3));
        Vector2f prev;
        bool first = true;
        for (const Vector2f& curr : _previousSolution) {
            if (first) {
                first = false;
            } else {
                painter.drawLine(QPointF(prev.x(), prev.y()),
                                 QPointF(curr.x(), curr.y()));
            }
            prev = curr;
        }

        //  draw cubic bezier interpolation of waypoints
        painter.setPen(QPen(Qt::darkBlue, 5));
        QPainterPath path(vecToPoint(_previousSolution[0]));

        Vector2f prevControlDiff = -_startVel * VelocityDrawingMultiplier;
        for (int i = 1; i < _previousSolution.size(); i++) {
            Vector2f waypoint = _previousSolution[i];
            Vector2f prevWaypoint = _previousSolution[i - 1];

            Vector2f controlDir;
            float controlLength;
            if (i == _previousSolution.size() - 1) {
                controlLength = _goalVel.norm() * VelocityDrawingMultiplier;
                controlDir = -_goalVel.normalized();
            } else {
                //  using first derivative heuristic from Sprunk 2008 to
                //  determine the
                //  distance of the control point from the waypoint
                Vector2f nextWaypoint = _previousSolution[i + 1];
                controlLength = 0.5 * min((waypoint - prevWaypoint).norm(),
                                          (nextWaypoint - waypoint).norm());
                controlDir = ((prevWaypoint - waypoint).normalized() -
                              (nextWaypoint - waypoint).normalized())
                                 .normalized();
            }

            Vector2f controlDiff = controlDir * controlLength;

            path.cubicTo(vecToPoint(prevWaypoint - prevControlDiff),
                         vecToPoint(waypoint + controlDiff),
                         vecToPoint(waypoint));

            prevControlDiff = controlDiff;
        }

        painter.drawPath(path);
    }

    //  draw waypoint cache
    if (_biRRT->waypoints().size() > 0) {
        float r = 2;  //  radius to draw waypoint dots

        painter.setPen(QPen(Qt::lightGray, 3));
        for (const Vector2f& waypoint : _biRRT->waypoints()) {
            painter.drawEllipse(QPointF(waypoint.x(), waypoint.y()), r, r);
        }
    }

    //  draw trees
    drawTree(painter, _biRRT->startTree(), _biRRT->startSolutionNode());
    drawTree(painter, _biRRT->goalTree(), _biRRT->goalSolutionNode(),
             Qt::darkGreen);

    //  draw start and goal states
    drawTerminalState(painter, _biRRT->startState(), _startVel, Qt::red);
    drawTerminalState(painter, _biRRT->goalState(), _goalVel, Qt::darkGreen);
}

void RRTWidget::drawTerminalState(QPainter& painter, const Vector2f& pos,
                                  const Vector2f& vel, const QColor& color) {
    //  draw point
    painter.setPen(QPen(color, 6));
    QPointF rootLoc(pos.x(), pos.y());
    painter.drawEllipse(rootLoc, 2, 2);

    Vector2f tipOffset = vel * VelocityDrawingMultiplier;
    Vector2f tipLocVec = pos + tipOffset;
    QPointF tipLoc(tipLocVec.x(), tipLocVec.y());

    //  draw arrow shaft
    painter.setPen(QPen(color, 3));
    painter.drawLine(rootLoc, tipLoc);

    //  draw arrow head
    Vector2f headBase = tipLocVec - tipOffset.normalized() * 4;
    Vector2f perp = Vector2f(-tipOffset.y(), tipOffset.x()).normalized();
    Vector2f tipLeftVec = headBase + perp * 4;
    Vector2f tipRightVec = headBase - perp * 4;
    QPointF trianglePts[] = {tipLoc, QPointF(tipLeftVec.x(), tipLeftVec.y()),
                             QPointF(tipRightVec.x(), tipRightVec.y())};
    painter.drawPolygon(trianglePts, 3);
}

void RRTWidget::drawTree(QPainter& painter, const Tree<Vector2f>& rrt,
                         const Node<Vector2f>* solutionNode, QColor treeColor,
                         QColor solutionColor) {
    //  node drawing radius
    const float r = 1;

    //  draw all the nodes and connections
    for (const Node<Vector2f>& node : rrt.allNodes()) {
        painter.setPen(QPen(treeColor, 1));
        QPointF loc = pointFromNode(&node);
        painter.drawEllipse(loc, r, r);

        if (node.parent()) {
            //  draw edge
            painter.setPen(QPen(treeColor, 1));
            QPointF parentLoc = pointFromNode(node.parent());
            painter.drawLine(loc, parentLoc);
        }
    }

    //  draw solution
    if (solutionNode) {
        painter.setPen(QPen(solutionColor, 2));

        const Node<Vector2f> *node = solutionNode,
                             *parent = solutionNode->parent();
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

bool RRTWidget::mouseInGrabbingRange(QMouseEvent* event, const Vector2f& pt) {
    float dx = event->pos().x() - pt.x();
    float dy = event->pos().y() - pt.y();
    return sqrtf(dx * dx + dy * dy) < 15;
}

void RRTWidget::mousePressEvent(QMouseEvent* event) {
    if (mouseInGrabbingRange(event, _biRRT->startState())) {
        _draggingItem = DraggingStart;
    } else if (mouseInGrabbingRange(event, _biRRT->goalState())) {
        _draggingItem = DraggingGoal;
    } else if (mouseInGrabbingRange(
                   event, _biRRT->startState() +
                              _startVel * VelocityDrawingMultiplier)) {
        _draggingItem = DraggingStartVel;
    } else if (mouseInGrabbingRange(event,
                                    _biRRT->goalState() +
                                        _goalVel * VelocityDrawingMultiplier)) {
        _draggingItem = DraggingGoalVel;
    } else {
        _editingObstacles = true;
        Vector2f pos = Vector2f(event->pos().x(), event->pos().y());
        Vector2i gridLoc =
            _stateSpace->obstacleGrid().gridSquareForLocation(pos);
        _erasingObstacles = _stateSpace->obstacleGrid().obstacleAt(gridLoc);

        //  toggle the obstacle state of clicked square
        _stateSpace->obstacleGrid().obstacleAt(gridLoc) = !_erasingObstacles;
        update();
    }
}

void RRTWidget::mouseMoveEvent(QMouseEvent* event) {
    Vector2f point(event->pos().x(), event->pos().y());

    if (_draggingItem == DraggingStart) {
        //  reset the tree with the new start pos
        _biRRT->setStartState(point);
    } else if (_draggingItem == DraggingGoal) {
        //  set the new goal point
        _biRRT->setGoalState(point);
    } else if (_draggingItem == DraggingStartVel) {
        _startVel = (point - _biRRT->startState()) / VelocityDrawingMultiplier;
    } else if (_draggingItem == DraggingGoalVel) {
        _goalVel = (point - _biRRT->goalState()) / VelocityDrawingMultiplier;
    } else if (_editingObstacles) {
        Vector2i gridLoc =
            _stateSpace->obstacleGrid().gridSquareForLocation(point);
        if (gridLoc[1] >= 0 &&
            gridLoc[1] < _stateSpace->obstacleGrid().discretizedHeight() &&
            gridLoc[0] >= 0 &&
            gridLoc[0] < _stateSpace->obstacleGrid().discretizedWidth()) {
            _stateSpace->obstacleGrid().obstacleAt(gridLoc) =
                !_erasingObstacles;
        }
    }

    if (_draggingItem != DraggingNone || _editingObstacles) update();
}

void RRTWidget::mouseReleaseEvent(QMouseEvent* event) {
    _draggingItem = DraggingNone;
    _editingObstacles = false;
}
