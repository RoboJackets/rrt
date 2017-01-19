import QtQuick 2.7
import QtQuick.Window 2.1
import QtQuick.Controls 1.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.2
import QtCharts 1.2

import RRTWidget 1.0

ApplicationWindow {
    title: "Interactive RRT"

    width: 300
    height: 300

    ColumnLayout {
        // main toolbar
        RowLayout {
            ColumnLayout {
                Button {
                    text: "Run"
                    // TODO: green
                    onClicked: rrt.run()
                }

                Button {
                    text: "Stop"
                    onClicked: rrt.stop()
                }
            }

            ColumnLayout {
                Button {
                    text: "Step"
                    onClicked: rrt.step()
                }

                Button {
                    text: "Step 100"
                    onClicked: rrt.stepBig()
                }
            }

            ColumnLayout {
                Button {
                    text: "Reset"
                    onClicked: rrt.reset()
                }

                Button {
                    text: "Clear Obstacles"
                    // TODO: red?
                    onClicked: rrt.clearObstacles()
                }
            }

            ColumnLayout {
                Label {
                    text: "Goal Bias: " + goalBiasSlider.value
                }

                Slider {
                    id: goalBiasSlider
                    minimumValue: 0
                    maximumValue: 1.0
                    stepSize: 0.05
                    value: rrt.goalBias
                    // TODO: bind to rrt
                }
            }

            ColumnLayout {
                Label {
                    text: "Waypoint Bias: " + waypointBiasSlider.value
                }

                Slider {
                    id: waypointBiasSlider
                    stepSize: 0.05
                    minimumValue: 0
                    maximumValue: 1.0
                    value: rrt.waypointBias
                    // TODO: bind to rrt
                }
            }

            ColumnLayout {
                Label {
                    text: "Step Size: "
                }

                SpinBox {
                    id: stepSize
                    decimals: 2
                    value: rrt.stepSize
                    // TODO: bind value
                }
            }
        }

        RRTWidget {
            id: rrt
            width: 800
            height: 600
        }
    }

    statusBar: StatusBar {
        RowLayout {
            anchors.fill: parent

            Label {
                text: "Iterations: " + rrt.iterations
            }
        }
    }


    Shortcut { sequence: 'r'; onActivated: rrt.run() }
    Shortcut { sequence: 's'; onActivated: rrt.stop() }
    Shortcut { sequence: 'c'; onActivated: rrt.reset() }
}
