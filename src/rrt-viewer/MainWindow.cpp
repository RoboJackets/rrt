
#include "MainWindow.hpp"


MainWindow::MainWindow() {
    _rrtWidget = new RRTWidget();

    setWindowTitle("Interactive RRT");

    QPushButton *step = new QPushButton(this);
    step->setText("Step");

    QPushButton *stepBig = new QPushButton(this);
    stepBig->setText("Step 100");
    
    QPushButton *reset = new QPushButton(this);
    reset->setText("Reset");

    QPushButton *clearObstacles = new QPushButton(this);
    clearObstacles->setText("Clear Obstacles");
    clearObstacles->setStyleSheet("background-color: red;");

    QCheckBox *bidirectional = new QCheckBox("Bidirectional");
    bidirectional->setChecked(_rrtWidget->bidirectional());

    QSlider *goalBias = new QSlider(Qt::Horizontal, this);
    goalBias->setTickPosition(QSlider::TicksBelow);
    goalBias->setMinimum(0);
    goalBias->setMaximum(100);
    goalBias->setTickInterval(10);

    _goalBiasLabel = new QLabel("Goal Bias: 0", this);

    QSlider *waypointBias = new QSlider(Qt::Horizontal, this);
    waypointBias->setTickPosition(QSlider::TicksBelow);
    waypointBias->setMinimum(0);
    waypointBias->setMaximum(100);
    waypointBias->setTickInterval(10);

    _waypointBiasLabel = new QLabel("Waypoint Bias: 0", this);

    QDoubleSpinBox *stepSizeBox = new QDoubleSpinBox(this);
    stepSizeBox->setMinimum(0.1);
    stepSizeBox->setMaximum(100);
    stepSizeBox->setValue(10);

    QLabel *stepSizeLabel = new QLabel("Step Size:");

    QGridLayout *layout = new QGridLayout();
    layout->addWidget(step, 1, 0);
    layout->addWidget(stepBig, 1, 1);
    layout->addWidget(reset, 1, 2);
    layout->addWidget(clearObstacles, 1, 3);
    layout->addWidget(bidirectional, 1, 4);
    layout->addWidget(goalBias, 1, 5);
    layout->addWidget(_goalBiasLabel, 0, 5);
    layout->addWidget(waypointBias, 1, 6);
    layout->addWidget(_waypointBiasLabel, 0, 6);
    layout->addWidget(stepSizeBox, 1, 7);
    layout->addWidget(stepSizeLabel, 0, 7);
    layout->addWidget(_rrtWidget, 2, 0, 1, 8);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);
    this->setCentralWidget(centralWidget);

    //  prevent the window from being resized
    setFixedSize(sizeHint());

    //  make the buttons do things
    connect(step, SIGNAL(clicked()), _rrtWidget, SLOT(slot_step()));
    connect(stepBig, SIGNAL(clicked()), _rrtWidget, SLOT(slot_stepBig()));
    connect(reset, SIGNAL(clicked()), _rrtWidget, SLOT(slot_reset()));
    connect(clearObstacles, SIGNAL(clicked()), _rrtWidget, SLOT(slot_clearObstacles()));
    connect(bidirectional, SIGNAL(stateChanged(int)), _rrtWidget, SLOT(slot_setBidirectional(int)));
    connect(goalBias, SIGNAL(valueChanged(int)), _rrtWidget, SLOT(slot_setGoalBias(int)));
    connect(goalBias, SIGNAL(valueChanged(int)), this, SLOT(slot_updateGoalBiasLabel(int)));
    connect(waypointBias, SIGNAL(valueChanged(int)), _rrtWidget, SLOT(slot_setWaypointBias(int)));
    connect(waypointBias, SIGNAL(valueChanged(int)), this, SLOT(slot_updateWaypointBiasLabel(int)));
    connect(stepSizeBox, SIGNAL(valueChanged(double)), _rrtWidget, SLOT(slot_setStepSize(double)));
}

void MainWindow::slot_updateGoalBiasLabel(int value) {
    _goalBiasLabel->setText(QString("Goal Bias: %1").arg(value / 100.0f));
}

void MainWindow::slot_updateWaypointBiasLabel(int value) {
    _waypointBiasLabel->setText(QString("Waypoint Bias: %1").arg(value / 100.0f));
}
