
#include "RRTWidget.hpp"
#include <2dplane/2dplane.hpp>

using namespace RRT;
using namespace Eigen;


RRTWidget::RRTWidget() {
    setFixedSize(600, 450);

    //  default to bidirectional
    _bidirectional = true;

    //  goal bias defaults to zero
    _goalBias = 0.0;

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

    slot_clearObstacles();
}

bool RRTWidget::bidirectional() const {
    return _bidirectional;
}

void RRTWidget::slot_reset() {
    resetTrees();
    update();
}

void RRTWidget::resetTrees() {
    //  begin fresh trees with the same starting points as before
    setupTree(&_startTree, _startTree->rootNode()->state());
    setupTree(&_goalTree, _goalTree->rootNode()->state());
}

void RRTWidget::slot_clearObstacles() {
    for (int x = 0; x < GridWidth; x++) {
        for (int y = 0; y < GridHeight; y++) {
            _blocked[x][y] = false;
        }
    }
    update();
}

void RRTWidget::slot_setBidirectional(int bidirectional) {
    if ((bool)bidirectional != _bidirectional) {
        _bidirectional = (bool)bidirectional;
        resetTrees();
        update();
    }
}

void RRTWidget::slot_setGoalBias(int bias) {
    _goalBias = (float)bias / 100.0f;
    if (_startTree) _startTree->setGoalBias(_goalBias);
    if (_goalTree) _goalTree->setGoalBias(_goalBias);
}

template<typename T>
bool inRange(T n, T min, T max) {
    return (n >= min) && (n <= max);
}

template<typename T>
void mySwap(T &a, T &b) {
    T tmp = a;
    a = b;
    b = tmp;
}

void RRTWidget::setupTree(Tree<Vector2f> **treePP, Vector2f start) {
    resetSolution();

    if (*treePP) delete *treePP;
    const float stepSize = 10;
    *treePP = TreeFor2dPlane(width(), height(), Vector2f(0,0), _stepSize);


    //  setup the callback to do collision checking for the new leg to be added to the tree
    (*treePP)->transitionValidator = [&](const Vector2f &from, const Vector2f &to) {
        Vector2f delta = to - from;

        //  get the corners of this segment in integer coordinates.  This limits our intersection test to only the boxes in that square
        int x1, y1; getIntCoordsForPt<Vector2f>(from, x1, y1);
        int x2, y2; getIntCoordsForPt<Vector2f>(to, x2, y2);


        //  order ascending
        if (x1 > x2) mySwap<int>(x1, x2);
        if (y1 > y2) mySwap<int>(y1, y2);

        QSizeF blockSize(rect().width() / GridWidth, rect().height() / GridHeight);

        //  check all squares from (x1, y1) to (x2, y2)
        for (int x = x1; x <= x2; x++) {
            for (int y = y1; y <= y2; y++) {
                if (_blocked[x][y]) {
                    //  there's an obstacle here, so check for intersection


                    //  the corners of this obstacle square
                    Vector2f ulCorner(x * blockSize.width(), y * blockSize.height());
                    Vector2f brCorner(ulCorner.x() + blockSize.width(), ulCorner.y() + blockSize.height());

                    if (delta.x() != 0) {
                        /**
                         * Find slope and y-intercept of the line passing through @from and @to.
                         * y1 = m*x1+b
                         * b = y1-m*x1
                         */
                        float slope = delta.y() / delta.x();
                        float b = to.y() - to.x()*slope;

                        /*
                         * First check intersection with the vertical segments of the box.  Use y=mx+b for the from-to line and plug in the x value for each wall
                         * If the corresponding y-value is within the y-bounds of the vertical segment, it's an intersection.
                         */
                        float yInt = slope*ulCorner.x() + b;
                        if (inRange<float>(yInt, ulCorner.y(), brCorner.y())) return false;
                        yInt = slope*brCorner.x() + b;
                        if (inRange<float>(yInt, ulCorner.y(), brCorner.y())) return false;

                        /*
                         * Check intersection with horizontal sides of box
                         * y = k;
                         * y = mx+b;
                         * mx+b = k;
                         * mx = k - b;
                         * (k - b) / m = x;  is x within the horizontal range of the box?
                         */
                        if (slope == 0) return false;
                        float xInt = (ulCorner.y() - b) / slope;
                        if (inRange<float>(xInt, ulCorner.x(), brCorner.x())) return false;
                        xInt = (brCorner.y() - b) / slope;
                        if (inRange<float>(xInt, ulCorner.x(), brCorner.x())) return false;
                    } else {
                        //  vertical line - slope undefined

                        //  see if it's within the x-axis bounds of this obstacle box
                        if (inRange<float>(from.x(), ulCorner.x(), brCorner.x())) {
                            //  order by y-value
                            //  note: @lower has a smaller value of y, but will appear higher visually on the screen due to qt's coordinate layout
                            Vector2f lower(from);
                            Vector2f higher(to);
                            if (higher.y() < lower.y()) swap<Vector2f>(lower, higher);

                            //  check for intersection based on y-values
                            if (lower.y() < ulCorner.y() && higher.y() > ulCorner.y()) return false;
                            if (lower.y() < brCorner.y() && higher.y() > brCorner.y()) return false;
                        }
                    }

                }
            }
        }


        return true;
    };

    (*treePP)->setup(start);

    (*treePP)->setGoalBias(_goalBias);

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

        if (_bidirectional) {
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
    }
    update();
}

Node<Vector2f> *RRTWidget::findBestPath(Vector2f targetState, Tree<Vector2f> *treeToSearch, int *depthOut) {
    Node<Vector2f> *bestNode = nullptr;
    int depth = INT_MAX;

    const float maxDist = 12;

    for (Node<Vector2f> *other : treeToSearch->allNodes()) {
        Vector2f delta = other->state() - targetState;
        if (magnitude(delta) < maxDist && other->depth() < depth) {
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
    int rectW = rect().width() / GridWidth, rectH = rect().height() / GridHeight;
    painter.setPen(QPen(Qt::black, 2));
    for (int x = 0; x < GridWidth; x++) {
        for (int y = 0; y < GridHeight; y++) {
            if (_blocked[x][y]) {
                painter.fillRect(x * rectW, y * rectH, rectW, rectH, Qt::SolidPattern);
            }
        }
    }

    //  draw @_startTree
    drawTree(painter, _startTree, _startSolutionNode);

    //  draw @_goalTree
    if (_bidirectional) {
        drawTree(painter, _goalTree, _goalSolutionNode, Qt::darkGreen);
    }

    //  draw root as a red dot
    if (_startTree->rootNode()) {
        painter.setPen(QPen (Qt::red, 6));
        QPointF rootLoc = pointFromNode(_startTree->rootNode());
        painter.drawEllipse(rootLoc, 2, 2);
    }

    //  draw goal as a green dot
    if (_goalTree->rootNode()) {
        QPointF goalLoc = pointFromNode(_goalTree->rootNode());
        painter.setPen(QPen(Qt::green, 6));
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
        int x, y; getIntCoordsForPt<QPointF>(event->pos(), x, y);
        _erasingObstacles = _blocked[x][y];

        //  toggle the blocked state of clicked square
        _blocked[x][y] = !_erasingObstacles;
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
        int x, y; getIntCoordsForPt<QPointF>(event->pos(), x, y);
        _blocked[x][y] = !_erasingObstacles;
        update();
    }
}

void RRTWidget::mouseReleaseEvent(QMouseEvent *event) {
    _draggingGoal = false;
    _draggingStart = false;
    _editingObstacles = false;
}
