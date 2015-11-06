import QtQuick 2.1
import QtQuick.Controls 1.0

ApplicationWindow {
    title: qsTr("Camera Acquisition")
    width: 640
    height: 480

    menuBar: MenuBar {
        Menu {
            title: qsTr("File")
            MenuItem {
                text: qsTr("Exit")
                onTriggered: Qt.quit();
            }
        }
        Menu {
            title: qsTr("Help")
            MenuItem {
                text: qsTr("Plugins...")
                //onTriggered: Qt.quit();
            }
            MenuItem {
                text: qsTr("About...")
                //onTriggered: Qt.quit();
            }
        }
    }

    SplitView {
        anchors.fill: parent
        orientation: Qt.Vertical

        ListView {
            height: 300
            model: cameraModel
            delegate: Item {
                width: parent.width
                height: 64
                Rectangle {
                    anchors.fill: parent
                    anchors.margins: 3
                    color: "gray"


                    Column {
                        anchors.fill: parent

                        Text {
                            text: camera.uuid
                        }

                        Row {
                            ComboBox {
                                model: camera.presets
                                enabled: !camera.running
                                onCurrentTextChanged: camera.loadPresetConfiguration(currentText)
                            }

                            Button {
                                text: camera.running ? "Stop" : "Start"
                                onClicked: {
                                    if (camera.running)
                                        camera.stopAcquisition();
                                    else
                                        camera.startAcquisition();
                                }
                            }
                        }


                    }
                }
            }
        }

        TableView {
            model: 50

            TableViewColumn {
                title: "Messages"
            }
        }
    }
}
