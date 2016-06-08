
#include <QtWidgets>
#include "MainWindow.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setOrganizationName("RoboJackets");
    app.setApplicationName("Interactive RRT");
    MainWindow mainWin;
    mainWin.show();

    return app.exec();
}
