import QtQuick 2.5
import QtQuick.Window 2.1
import QtQuick.Controls 1.2
import QtQuick.Controls.Styles 1.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.2

import RRTWidget 1.0

ApplicationWindow {
    title: "Interactive RRT"

    // makes window floating by default in tiling window managers
    modality: Qt.WindowModal

    ColumnLayout {
        anchors.fill: parent;

        // main toolbar
        Row{
            // Add back when we use QtQuick 2.7+
            /* padding: 5 */
            /* spacing: 5 */
            Layout.alignment: Qt.AlignTop

            ColumnLayout {
                Button {
                    onClicked: rrt.run()
                    text: "Run"
                    Rectangle {
                        anchors.fill: parent
                        anchors.margins: 1
                        color: "green"
                        opacity : .5
                    }
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
                    onClicked: rrt.clearObstacles()

                    Rectangle {
                        anchors.fill: parent
                        anchors.margins: 1
                        color: "red"
                        opacity : .5
                    }
                }
            }

            ColumnLayout {
                Label {
                    text: "Goal Bias: " + goalBiasSlider.value.toFixed(2)
                }

                Slider {
                    id: goalBiasSlider
                    minimumValue: 0
                    maximumValue: 1.0
                    stepSize: 0.05
                    tickmarksEnabled: true
                    value: 0
                }
            }

            ColumnLayout {
                Label {
                    text: "Waypoint Bias: " + waypointBiasSlider.value.toFixed(2)
                }

                Slider {
                    id: waypointBiasSlider
                    stepSize: 0.05
                    minimumValue: 0
                    maximumValue: 1.0
                    tickmarksEnabled: true
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

            ColumnLayout {
                Label {
                    text: "Adaptive\nStepsize"
                }

                CheckBox {
                    id: ascCheckbox
                    checked: false
                }
            }
        }

        // draw and interact with the rrt
        RRTWidget {
            // Fixed size
            Layout.maximumHeight: 600
            Layout.maximumWidth: 800
            Layout.minimumHeight: 600
            Layout.minimumWidth: 800

            id: rrt
        }
    }

    // bottom bar
    statusBar: StatusBar {
        RowLayout {
            id: statusBarLayout
            anchors.fill: parent

            Label {
                text: "Iterations: " + rrt.iterations
                anchors.right: statusBarLayout.right
            }
        }
    }

    // update rrt values when the ui controls change
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
    Binding {
        target: rrt
        property: "ascEnabled"
        value: ascCheckbox.checked
    }

    // keyboard shortcuts
    Shortcut { sequence: 'r'; onActivated: rrt.run() }
    Shortcut { sequence: 's'; onActivated: rrt.stop() }
    Shortcut { sequence: 'c'; onActivated: rrt.reset() }
    Shortcut{ sequence: 'o'; onActivated: rrt.clearObstacles() }
    Shortcut{ sequence: 't'; onActivated: rrt.step() }
    Shortcut{ sequence: 'b'; onActivated: rrt.stepBig() }
}
