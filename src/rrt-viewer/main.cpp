
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QtQuick>
#include <QtWidgets>
#include <iostream>
#include "RRTWidget.hpp"

using namespace std;

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setOrganizationName("RoboJackets");
    app.setApplicationName("Interactive RRT");

    // load qml
    QQmlApplicationEngine engine(nullptr);
    engine.rootContext()->setContextProperty("main", &engine);

    qmlRegisterType<RRTWidget>("RRTWidget", 1, 0, "RRTWidget");

    // load main.qml from qt resources - baked into binary through cmake
    engine.load((QUrl("qrc:///main.qml")));

    // get reference to main window from qml
    auto win = static_cast<QQuickView*>(engine.rootObjects()[0]);
    if (!win) {
        cerr << "Failed to load qml" << endl;
        exit(1);
    }

    win->show();

    return app.exec();
}
