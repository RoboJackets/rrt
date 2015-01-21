#pragma once

#include <QtWidgets>
#include <2dplane/ObstacleGrid.hpp>


class RRTWidget : public QWidget {
    Q_OBJECT

public:
    RRTWidget(QWidget *parent = nullptr);

    /// override this and call emit signal_stepped(int) and update() at the end
    virtual void step(int numTimes) = 0;

    virtual bool hasSolution() const = 0;

    void paintEvent(QPaintEvent *p);
    void drawObstacles(QPainter &painter, const ObstacleGrid &obstacleGrid);


private slots:
    void slot_run();
    void run_step();
    void slot_stop();

    void slot_step();
    void slot_stepBig();

    /// this should emit signal_stepped() at the end to update the iteration count
    /// also call update()
    virtual void slot_reset() = 0;
    virtual void slot_clearObstacles() = 0;

    /// these two slots are passed an int from 0 to 100, so you'll want to scale it to be from 0 to 1
    virtual void slot_setGoalBias(int bias) = 0;        //  bias is from 0 to 100
    virtual void slot_setWaypointBias(int bias) = 0;    //  bias is from 0 to 100
    

    virtual void slot_setStepSize(double step) = 0;


signals:
    void signal_stepped(int iterationCount);


private:
    QTimer *_runTimer;
};
