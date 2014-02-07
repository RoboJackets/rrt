
#include "MainWindow.hpp"


MainWindow::MainWindow() {
	setWindowTitle("Interactive RRT");

	QPushButton *step = new QPushButton(this);
	step->setText("Step");
	
	QPushButton *reset = new QPushButton(this);
	reset->setText("Reset");

	QPushButton *stepBig = new QPushButton(this);
	stepBig->setText("Step 100");

	_rrtWidget = new RRTWidget();

	QGridLayout *layout = new QGridLayout();
	layout->addWidget(step, 0, 0);
	layout->addWidget(stepBig, 0, 1);
	layout->addWidget(reset, 0, 2);
	layout->addWidget(_rrtWidget, 1, 0, 1, 3);

	QWidget *centralWidget = new QWidget(this);
	centralWidget->setLayout(layout);
	this->setCentralWidget(centralWidget);

    setFixedSize(sizeHint());

	connect(step, SIGNAL(released()), _rrtWidget, SLOT(slot_step()));
	connect(reset, SIGNAL(released()), _rrtWidget, SLOT(slot_reset()));
	connect(stepBig, SIGNAL(released()), _rrtWidget, SLOT(slot_stepBig()));
}
