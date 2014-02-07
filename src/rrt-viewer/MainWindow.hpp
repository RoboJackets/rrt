
#include <QtWidgets>
#include "RRTWidget.hpp"


class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	MainWindow();

private:
	RRTWidget *_rrtWidget;
};
