
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

    QCheckBox *bidirectional = new QCheckBox("Bidirectional");
    bidirectional->setChecked(_rrtWidget->bidirectional());

    QSlider *goalBias = new QSlider(Qt::Horizontal, this);
    goalBias->setTickPosition(QSlider::TicksBelow);
    goalBias->setMinimum(0);
    goalBias->setMaximum(100);
    goalBias->setTickInterval(10);

    _goalBiasLabel = new QLabel("Goal Bias: 0", this);

    QGridLayout *layout = new QGridLayout();
    layout->addWidget(step, 1, 0);
    layout->addWidget(stepBig, 1, 1);
    layout->addWidget(reset, 1, 2);
    layout->addWidget(clearObstacles, 1, 3);
    layout->addWidget(bidirectional, 1, 4);
    layout->addWidget(goalBias, 1, 5);
    layout->addWidget(_goalBiasLabel, 0, 5);
    layout->addWidget(_rrtWidget, 2, 0, 1, 6);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);
    this->setCentralWidget(centralWidget);

    //  prevent the window from being resized
    setFixedSize(sizeHint());

    //  make the buttons do things
    connect(step, SIGNAL(released()), _rrtWidget, SLOT(slot_step()));
    connect(stepBig, SIGNAL(released()), _rrtWidget, SLOT(slot_stepBig()));
    connect(reset, SIGNAL(released()), _rrtWidget, SLOT(slot_reset()));
    connect(clearObstacles, SIGNAL(released()), _rrtWidget, SLOT(slot_clearObstacles()));
    connect(bidirectional, SIGNAL(stateChanged(int)), _rrtWidget, SLOT(slot_setBidirectional(int)));
    connect(goalBias, SIGNAL(valueChanged(int)), _rrtWidget, SLOT(slot_setGoalBias(int)));
    connect(goalBias, SIGNAL(valueChanged(int)), this, SLOT(slot_updateGoalBiasLabel(int)));
}

void MainWindow::slot_updateGoalBiasLabel(int value) {
    _goalBiasLabel->setText(QString("Goal Bias: %1").arg(value / 100.0f));
}
