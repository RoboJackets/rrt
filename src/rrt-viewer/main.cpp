
#include "MainWindow.hpp"
#include <QtWidgets>


int main(int argc, char **argv) {
	// Q_INIT_RESOURCE(application);

	QApplication app(argc, argv);
	app.setOrganizationName("RoboJackets");
	app.setApplicationName("RRT Viewer");
	MainWindow mainWin;
	mainWin.show();

	return app.exec();
}
