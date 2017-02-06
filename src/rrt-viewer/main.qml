import QtQuick 2.7
import QtQuick.Window 2.1
import QtQuick.Controls 1.2
import QtQuick.Controls.Styles 1.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.2

import RRTWidget 1.0

ApplicationWindow {
    title: "Interactive RRT"

    // Fixed size
    maximumHeight: 689
    maximumWidth: 816
    minimumHeight: 689
    minimumWidth: 816

    ColumnLayout {
        anchors.fill: parent;

        // main toolbar
        Row{
            padding: 5
            spacing: 5
            Layout.alignment: Qt.AlignTop

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
                    value: 0
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
                    value: 0
                }
            }

            ColumnLayout {
                Label {
                    text: "Step Size: "
                }

                SpinBox {
                    id: stepSizeBox
                    decimals: 1
                    value: 10
                }
            }
        }

        RRTWidget {
            id: rrt
            Layout.fillHeight: true
            Layout.fillWidth: true
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

    Binding {
        target: rrt
        property: "goalBias"
        value: goalBiasSlider.value
    }

    Binding {
        target: rrt
        property: "waypointBias"
        value: waypointBiasSlider.value
    }

    Binding {
        target: rrt
        property: "stepSize"
        value: stepSizeBox.value
    }

    Shortcut { sequence: 'r'; onActivated: rrt.run() }
    Shortcut { sequence: 's'; onActivated: rrt.stop() }
    Shortcut { sequence: 'c'; onActivated: rrt.reset() }
}
