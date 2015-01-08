
#include "RRTWidget.hpp"
#include <planning/Path.hpp>
#include <2dplane/2dplane.hpp>

using namespace RRT;
using namespace Eigen;


RRTWidget::RRTWidget() {
    setFixedSize(800, 600);

    _environment = make_shared<GridStateSpace>(rect().width(),
                                        rect().height(),
                                        40,
                                        30);

    //  biases default to zero
    _goalBias = 0.0;
    _waypointBias = 0.0;
    _waypointCacheMaxSize = 15;

    _stepSize = 10;

    //  reset
    resetSolution();

    //  setup trees
    _startTree = nullptr;
    _goalTree = nullptr;
    setupTree(&_startTree, Vector2f(50, 50));
    setupTree(&_goalTree, Vector2f(width() / 2.0, height() / 2.0));

    //  register for mouse events
    setMouseTracking(true);
    _draggingStart = false;
    _draggingGoal = false;
}

void RRTWidget::getSolution(vector<Vector2f> &solutionOut) {
    if (_startSolutionNode && _goalSolutionNode) {
        //  add nodes from start tree starting at the end, then working back to the root
        //  reverse at the end to get them in the right order (so the root is index 0)
        for (const RRT::Node<Eigen::Vector2f> *n = _startSolutionNode; n != nullptr; n = n->parent()) {
            solutionOut.push_back(n->state());
        }
        reverse(solutionOut.begin(), solutionOut.end());

        for (const RRT::Node<Eigen::Vector2f> *n = _goalSolutionNode; n != nullptr; n = n->parent()) {
            solutionOut.push_back(n->state());
        }
    }
}

void RRTWidget::slot_reset() {
    //  store waypoint cache

    vector<Vector2f> waypoints;
    if (_startSolutionNode && _goalSolutionNode) {
        waypoints = _previousSolution;
        if (waypoints.size() > 0) {
            //  don't keep the start or end states
            waypoints.erase(waypoints.begin());
            waypoints.erase(waypoints.end());

            //  down-sample
            Planning::DownSampleVector<Vector2f>(waypoints, _waypointCacheMaxSize);
        }
    } else {
        _previousSolution.clear();
    }

    resetTrees();

    _startTree->setWaypoints(waypoints);
    _goalTree->setWaypoints(waypoints);

    update();
}

void RRTWidget::resetTrees() {
    //  begin fresh trees with the same starting points as before
    setupTree(&_startTree, _startTree->rootNode()->state());
    setupTree(&_goalTree, _goalTree->rootNode()->state());
}

void RRTWidget::slot_clearObstacles() {
    _environment->clearObstacles();

    update();
}

void RRTWidget::slot_setGoalBias(int bias) {
    _goalBias = (float)bias / 100.0f;
    if (_startTree) _startTree->setGoalBias(_goalBias);
    if (_goalTree) _goalTree->setGoalBias(_goalBias);
}

void RRTWidget::slot_setWaypointBias(int bias) {
    _waypointBias = (float)bias / 100.0f;
    if (_startTree) _startTree->setWaypointBias(_waypointBias);
    if (_goalTree) _goalTree->setWaypointBias(_waypointBias);
}

void RRTWidget::setupTree(Tree<Vector2f> **treePP, Vector2f start) {
    resetSolution();

    if (*treePP) delete *treePP;
    const float stepSize = 10;
    *treePP = TreeFor2dPlane(_environment, Vector2f(0,0), _stepSize);

    (*treePP)->setStartState(start);

    (*treePP)->setGoalBias(_goalBias);
    (*treePP)->setWaypointBias(_waypointBias);

    updateTreeGoals();
}

void RRTWidget::updateTreeGoals() {
    if (_goalTree && _startTree && _goalTree->rootNode() && _startTree->rootNode()) {
        _startTree->setGoalState(_goalTree->startState());
        _goalTree->setGoalState(_startTree->startState());
    }
}

void RRTWidget::updateStepSizes() {
    if (_startTree) _startTree->setStepSize(_stepSize);
    if (_goalTree) _goalTree->setStepSize(_stepSize);
}

void RRTWidget::resetSolution() {
    _startSolutionNode = nullptr;
    _goalSolutionNode = nullptr;
    _solutionLength = INT_MAX;
}

void RRTWidget::slot_step() {
    step(1);
}

void RRTWidget::slot_stepBig() {
    step(100);
}

void RRTWidget::slot_setStepSize(double step) {
    _stepSize = step;
    updateStepSizes();
}

void RRTWidget::step(int numTimes) {
    for (int i = 0; i < numTimes; i++) {
        Node<Vector2f> *newNode = _startTree->grow();
        
        int depth;
        Node<Vector2f> *otherNode;

        if (newNode) {
            otherNode = findBestPath(newNode->state(), _goalTree, &depth);
            if (otherNode && depth + newNode->depth() < _solutionLength) {
                _startSolutionNode = newNode;
                _goalSolutionNode = otherNode;
                _solutionLength = newNode->depth() + depth;
            }
        }

        newNode = _goalTree->grow();

        if (newNode) {
            otherNode = findBestPath(newNode->state(), _startTree, &depth);
            if (otherNode && depth + newNode->depth() < _solutionLength) {
                _startSolutionNode = otherNode;
                _goalSolutionNode = newNode;
                _solutionLength = newNode->depth() + depth;
            }
        }
    }


    //  store solution
    _previousSolution.clear();
    if (_solutionLength != INT_MAX) {
        getSolution(_previousSolution);
        Planning::SmoothPath<Vector2f>(_previousSolution, _startTree->stateSpace());
    }

    update();
}

Node<Vector2f> *RRTWidget::findBestPath(Vector2f targetState, Tree<Vector2f> *treeToSearch, int *depthOut) {
    Node<Vector2f> *bestNode = nullptr;
    int depth = INT_MAX;

    const float maxDist = 12;

    for (Node<Vector2f> *other : treeToSearch->allNodes()) {
        Vector2f delta = other->state() - targetState;
        if (delta.norm() < maxDist && other->depth() < depth) {
            bestNode = other;
            depth = other->depth();
        }
    }

    if (depthOut) *depthOut = depth;

    return bestNode;
}

QPointF RRTWidget::pointFromNode(const Node<Vector2f> *n) {
    return QPointF(n->state().x(), n->state().y());
}

void RRTWidget::paintEvent(QPaintEvent *p) {
    QPainter painter(this);

    //  draw black border around widget
    painter.setPen(QPen (Qt::black, 3));
    painter.drawRect(rect());

    //  draw obstacles
    int rectW = rect().width() / _environment->discretizedWidth(), rectH = rect().height() / _environment->discretizedHeight();
    painter.setPen(QPen(Qt::black, 2));
    for (int x = 0; x < _environment->discretizedWidth(); x++) {
        for (int y = 0; y < _environment->discretizedHeight(); y++) {
            if (_environment->obstacleAt(x, y)) {
                painter.fillRect(x * rectW, y * rectH, rectW, rectH, Qt::SolidPattern);
            }
        }
    }


    //  draw previous solution
    if (_previousSolution.size() > 0) {
        painter.setPen(QPen(Qt::yellow, 3));
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
    }


    //  draw waypoint cache
    if (_startTree->waypoints().size() > 0) {
        float r = 2;    //  radius to draw waypoint dots

        painter.setPen(QPen(Qt::lightGray, 3));
        for (const Vector2f &waypoint : _startTree->waypoints()) {
            painter.drawEllipse(QPointF(waypoint.x(), waypoint.y()), r, r);
        }
    }

    //  draw @_startTree
    drawTree(painter, _startTree, _startSolutionNode);

    //  draw @_goalTree
    drawTree(painter, _goalTree, _goalSolutionNode, Qt::darkGreen);

    //  draw root as a red dot
    if (_startTree->rootNode()) {
        painter.setPen(QPen (Qt::red, 6));
        QPointF rootLoc = pointFromNode(_startTree->rootNode());
        painter.drawEllipse(rootLoc, 2, 2);
    }

    //  draw goal as a green dot
    if (_goalTree->rootNode()) {
        QPointF goalLoc = pointFromNode(_goalTree->rootNode());
        painter.setPen(QPen(Qt::darkGreen, 6));
        painter.drawEllipse(goalLoc, 2, 2);
    }
}

void RRTWidget::drawTree(QPainter &painter,
    const Tree<Vector2f> *rrt,
    const Node<Vector2f> *solutionNode,
    QColor treeColor,
    QColor solutionColor)
{
    //  node drawing radius
    const float r = 1;

    //  draw all the nodes and connections
    for (const Node<Vector2f> *node : rrt->allNodes()) {
        painter.setPen(QPen (treeColor, 1));
        QPointF loc = pointFromNode(node);
        painter.drawEllipse(loc, r, r);

        if (node->parent()) {
            //  draw edge
            painter.setPen(QPen(treeColor, 1));
            QPointF parentLoc = pointFromNode(node->parent());
            painter.drawLine(loc, parentLoc);
        }
    }

    //  draw solution
    if (solutionNode) {
        painter.setPen(QPen(solutionColor, 2));

        const Node<Vector2f> *node = solutionNode, *parent = solutionNode->parent();
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
    float dx = event->pos().x() - pt.x();
    float dy = event->pos().y() - pt.y();
    return sqrtf( dx*dx + dy*dy ) < 15;
}

void RRTWidget::mousePressEvent(QMouseEvent *event) {
    if (mouseInGrabbingRange(event, _startTree->rootNode()->state())) {
        _draggingStart = true;
    } else if (mouseInGrabbingRange(event, _goalTree->rootNode()->state())) {
        _draggingGoal = true;
    } else {
        _editingObstacles = true;
        Vector2f pos = Vector2f(event->pos().x(), event->pos().y());
        Vector2i gridLoc = _environment->gridSquareForState(pos);
        _erasingObstacles = _environment->obstacleAt(gridLoc);

        //  toggle the obstacle state of clicked square
        _environment->obstacleAt(gridLoc) = !_erasingObstacles;
        update();
    }
}

void RRTWidget::mouseMoveEvent(QMouseEvent *event) {
    Vector2f point(event->pos().x(), event->pos().y());

    if (_draggingStart) {
        //  reset the tree with the new start pos
        setupTree(&_startTree, point);
        updateTreeGoals();
        update();
    } else if (_draggingGoal) {
        //  set the new goal point
        setupTree(&_goalTree, point);
        updateTreeGoals();
        update();
    } else if (_editingObstacles) {
        Vector2i gridLoc = _environment->gridSquareForState(point);
        _environment->obstacleAt(gridLoc) = !_erasingObstacles;
        update();
    }
}

void RRTWidget::mouseReleaseEvent(QMouseEvent *event) {
    _draggingGoal = false;
    _draggingStart = false;
    _editingObstacles = false;
}
