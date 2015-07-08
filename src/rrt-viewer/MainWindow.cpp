
#include "MainWindow.hpp"


MainWindow::MainWindow() {
    _rrtWidget = new RRTWidget();

    setWindowTitle("Interactive RRT");

    _iterationCountLabel = new QLabel(this);
    _iterationCountLabel->setText("Iterations: 0");
    statusBar()->addPermanentWidget(_iterationCountLabel);

    QPushButton *run = new QPushButton(this);
    run->setText("Run");
    run->setStyleSheet("background-color: green;");

    QPushButton *runFast = new QPushButton(this);
    runFast->setText("Run Fast");
    runFast->setStyleSheet("background-color: darkGreen;");

    QPushButton *stop = new QPushButton(this);
    stop->setText("Stop");

    QPushButton *step = new QPushButton(this);
    step->setText("Step");

    QPushButton *stepBig = new QPushButton(this);
    stepBig->setText("Step 100");
    
    QPushButton *reset = new QPushButton(this);
    reset->setText("Reset");

    QPushButton *clearObstacles = new QPushButton(this);
    clearObstacles->setText("Clear Obstacles");
    clearObstacles->setStyleSheet("background-color: red;");

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
    stepSizeBox->setMinimum(0.01);
    stepSizeBox->setMaximum(1);
    stepSizeBox->setValue(0.1);

    QLabel *stepSizeLabel = new QLabel("Step Size:");

    QDoubleSpinBox *angleDiffDecayFactorBox = new QDoubleSpinBox(this);
    angleDiffDecayFactorBox->setMinimum(1);
    angleDiffDecayFactorBox->setMaximum(5);
    angleDiffDecayFactorBox->setValue(1.1);

    QLabel *angleDiffDecayFactorLabel = new QLabel("Angle Decay factor");

    QGridLayout *layout = new QGridLayout();
    layout->addWidget(run, 0, 0);
    layout->addWidget(stop, 1, 0);
    layout->addWidget(runFast, 2, 0);
    layout->addWidget(step, 0, 1);
    layout->addWidget(stepBig, 1, 1);
    layout->addWidget(reset, 0, 2);
    layout->addWidget(clearObstacles, 1, 2);
    layout->addWidget(goalBias, 1, 3);
    layout->addWidget(_goalBiasLabel, 0, 3);
    layout->addWidget(waypointBias, 1, 4);
    layout->addWidget(_waypointBiasLabel, 0, 4);
    layout->addWidget(stepSizeBox, 1, 5);
    layout->addWidget(stepSizeLabel, 0, 5);
    layout->addWidget(angleDiffDecayFactorBox, 1, 6);
    layout->addWidget(angleDiffDecayFactorLabel, 0, 6);
    layout->addWidget(_rrtWidget, 3, 0, 1, 7);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);
    this->setCentralWidget(centralWidget);

    //  prevent the window from being resized
    setFixedSize(sizeHint());

    //  make the buttons do things
    connect(run, SIGNAL(clicked()), _rrtWidget, SLOT(slot_run()));
    connect(runFast, SIGNAL(clicked()), _rrtWidget, SLOT(slot_runFast()));
    connect(stop, SIGNAL(clicked()), _rrtWidget, SLOT(slot_stop()));
    connect(step, SIGNAL(clicked()), _rrtWidget, SLOT(slot_step()));
    connect(stepBig, SIGNAL(clicked()), _rrtWidget, SLOT(slot_stepBig()));
    connect(reset, SIGNAL(clicked()), _rrtWidget, SLOT(slot_reset()));
    connect(clearObstacles, SIGNAL(clicked()), _rrtWidget, SLOT(slot_clearObstacles()));
    connect(goalBias, SIGNAL(valueChanged(int)), _rrtWidget, SLOT(slot_setGoalBias(int)));
    connect(goalBias, SIGNAL(valueChanged(int)), this, SLOT(slot_updateGoalBiasLabel(int)));
    connect(waypointBias, SIGNAL(valueChanged(int)), _rrtWidget, SLOT(slot_setWaypointBias(int)));
    connect(waypointBias, SIGNAL(valueChanged(int)), this, SLOT(slot_updateWaypointBiasLabel(int)));
    connect(stepSizeBox, SIGNAL(valueChanged(double)), _rrtWidget, SLOT(slot_setStepSize(double)));
    connect(angleDiffDecayFactorBox, SIGNAL(valueChanged(double)), _rrtWidget, SLOT(slot_setMaxAngleDiffDecayFactor(double)));
    connect(_rrtWidget, SIGNAL(signal_stepped(int)), this, SLOT(slot_updateIterationCount(int)));

    //  keyboard shortcuts
    new QShortcut(QKeySequence(Qt::Key_R), _rrtWidget, SLOT(slot_run()));
    new QShortcut(QKeySequence(Qt::Key_S), _rrtWidget, SLOT(slot_stop()));
    new QShortcut(QKeySequence(Qt::Key_C), _rrtWidget, SLOT(slot_reset()));
    new QShortcut(QKeySequence(Qt::Key_F), _rrtWidget, SLOT(slot_runFast()));
}

void MainWindow::slot_updateGoalBiasLabel(int value) {
    _goalBiasLabel->setText(QString("Goal Bias: %1").arg(value / 100.0f));
}

void MainWindow::slot_updateWaypointBiasLabel(int value) {
    _waypointBiasLabel->setText(QString("Waypoint Bias: %1").arg(value / 100.0f));
}

void MainWindow::slot_updateIterationCount(int iterationCount) {
    _iterationCountLabel->setText(QString("Iterations: %1").arg(iterationCount));
}
