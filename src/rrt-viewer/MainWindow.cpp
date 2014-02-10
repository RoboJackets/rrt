
#include "MainWindow.hpp"


MainWindow::MainWindow() {
	setWindowTitle("Interactive RRT");

	QPushButton *step = new QPushButton(this);
	step->setText("Step");

	QPushButton *stepBig = new QPushButton(this);
	stepBig->setText("Step 100");
	
	QPushButton *reset = new QPushButton(this);
	reset->setText("Reset");

	QPushButton *clearObstacles = new QPushButton(this);
	clearObstacles->setText("Clear Obstacles");

	_rrtWidget = new RRTWidget();

	QGridLayout *layout = new QGridLayout();
	layout->addWidget(step, 0, 0);
	layout->addWidget(stepBig, 0, 1);
	layout->addWidget(reset, 0, 2);
	layout->addWidget(clearObstacles, 0, 3);
	layout->addWidget(_rrtWidget, 1, 0, 1, 4);

	QWidget *centralWidget = new QWidget(this);
	centralWidget->setLayout(layout);
	this->setCentralWidget(centralWidget);

    setFixedSize(sizeHint());

	connect(step, SIGNAL(released()), _rrtWidget, SLOT(slot_step()));
	connect(stepBig, SIGNAL(released()), _rrtWidget, SLOT(slot_stepBig()));
	connect(reset, SIGNAL(released()), _rrtWidget, SLOT(slot_reset()));
	connect(clearObstacles, SIGNAL(released()), _rrtWidget, SLOT(slot_clearObstacles()));
}
