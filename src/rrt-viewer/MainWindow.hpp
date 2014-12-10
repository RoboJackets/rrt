
#include <QtWidgets>
#include "RRTWidget.hpp"

/**
 * This window subclass displays the Interactive RRT 'applet'.
 * It has an RRTWidget that takes up most of the space and a
 * buttons for interacting with the RRT.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow();

private:
    RRTWidget *_rrtWidget;
};
