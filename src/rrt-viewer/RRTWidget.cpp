#include "RRTWidget.hpp"


RRTWidget::RRTWidget(QWidget *parent) : QWidget(parent) {
    _runTimer = nullptr;

    setFixedSize(800, 600);
}

void RRTWidget::slot_step() {
    step(1);
}

void RRTWidget::slot_stepBig() {
    step(100);
}

void RRTWidget::slot_run() {
    if (!_runTimer) {
        _runTimer = new QTimer(this);
        connect(_runTimer, SIGNAL(timeout()), this, SLOT(run_step()));
        _runTimer->start(0);
    }
}

void RRTWidget::run_step() {
    if (hasSolution()) {
        slot_stop();
    } else {
        step(1);
    }
}

void RRTWidget::slot_stop() {
    if (_runTimer) {
        delete _runTimer;
        _runTimer = nullptr;
    }
}

void RRTWidget::paintEvent(QPaintEvent *p) {
    QPainter painter(this);

    //  draw black border around widget
    painter.setPen(QPen (Qt::black, 3));
    painter.drawRect(rect());
}

void RRTWidget::drawObstacles(QPainter &painter, const ObstacleGrid &obstacleGrid) {
    //  draw obstacles
    int rectW = rect().width() / obstacleGrid.discretizedWidth(), rectH = rect().height() / obstacleGrid.discretizedHeight();
    painter.setPen(QPen(Qt::black, 2));
    for (int x = 0; x < obstacleGrid.discretizedWidth(); x++) {
        for (int y = 0; y < obstacleGrid.discretizedHeight(); y++) {
            if (obstacleGrid.obstacleAt(x, y)) {
                painter.fillRect(x * rectW, y * rectH, rectW, rectH, Qt::SolidPattern);
            }
        }
    }
}