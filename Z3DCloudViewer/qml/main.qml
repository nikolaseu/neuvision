import Qt3D.Core 2.12 as Q3D
import Qt3D.Extras 2.12
import Qt3D.Input 2.12
import Qt3D.Render 2.12

import QtQuick 2.10
import QtQuick.Controls 2.3
import QtQuick.Dialogs 1.3
import QtQuick.Layouts 1.3
import QtQuick.Scene3D 2.0

import Qt.labs.settings 1.0

import Z3D.ZPointCloud 1.0

ApplicationWindow {
    id: window
    title: qsTr("Point cloud viewer")
    width: 1200
    height: 800
    visible: true

    color: "#ddd"

    property int maxRecentFiles: 20
    property string lastOpenedFile
    property var recentFiles: []

    onLastOpenedFileChanged: {
        console.log("lastOpenedFileChanged:", lastOpenedFile)
        if (!lastOpenedFile) {
            return;
        }

        const fileIndex = recentFiles.indexOf(lastOpenedFile);
        if (fileIndex < 0) {
            console.log("adding lastOpenedFile:", lastOpenedFile, "because it's not in the recent files list:", recentFiles)
            // it's not in the recent files list, we'll just add it
        }
        else {
            // remove it from where it was, we'll add it to the beginning later
            recentFiles.splice(fileIndex, 1);
        }
        // add it to the front
        // have to concat and reassign to make sure it's saved in settings :(
        recentFiles = [lastOpenedFile].concat(recentFiles);
    }

    onRecentFilesChanged: {
        console.log("recentFilesChanged:", recentFiles);
        if (recentFiles.length > maxRecentFiles) {
            console.log("too much recent files, removing old ones...");
            recentFiles = recentFiles.slice(0, maxRecentFiles);
        }
    }

    Settings {
        id: recentFilesSettings
        category: "Recent"

        property alias recentFiles: window.recentFiles
    }

    FileDialog {
        id: fileDialog
        title: qsTr("Open a point cloud")
        folder: shortcuts.home
        nameFilters: ["PLY files (*.ply)", "STL files (*.stl)", "PCD files (*.pcd)"]
        onAccepted: {
            const fileName = fileDialog.fileUrl;
            pointCloudReader.source = fileName;
            window.lastOpenedFile = fileName;
        }
    }

    PointCloudReader {
        id: pointCloudReader
        //filename: "/Users/nikolaseu/Downloads/something.ply"
    }

    Item {
        anchors.fill: parent

        Scene3D {
            id: scene3d
            anchors.fill: parent

            aspects: ["input", "logic"]
            cameraAspectRatioMode: Scene3D.AutomaticAspectRatio
            focus: true
            multisample: false // faster

            Q3D.Entity {
                id: sceneRoot

                BasicCamera {
                    id: mainCamera
                    aspectRatio: scene3d.width/scene3d.height
                }

                DefaultCameraController {
                    camera: mainCamera
                }

                components: [
                    RenderSettings {
                        id: renderSettings
                        activeFrameGraph: ForwardRenderer {
                            id: renderer
                            camera: mainCamera
                            clearColor: window.color
                        }
                    },
                    InputSettings {
                        eventSource: window
                        enabled: true
                    }
                ]

                PointCloudEntity {
                    id: pointCloud
                    pointCloud: pointCloudReader.pointCloud

                    pointSize: pointSizeSlider.value
                    lightPosition: mainCamera.position
                    ambient: ambientSlider.value
                    diffuse: diffuseSlider.value
                    specular: specularSlider.value
                    shininess: shininessSlider.value

                    transform: Q3D.Transform {
                        // center in 0,0,0
                        translation: Qt.vector3d(-pointCloud.pointCloud.center.x, -pointCloud.pointCloud.center.y, -pointCloud.pointCloud.center.z)
                    }
                }
            }
        }

        RowLayout {
            id: menuLayout
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.margins: 4
            opacity: mouseAreaForMenuLayout.containsMouse ? 1 : 0.3
            visible: pointCloudReader.pointCloud

            Behavior on opacity {
                OpacityAnimator {
                    duration: 100
                }
            }

            Button {
                text: "Open file..."
                onClicked: fileDialog.open()
            }

            Button {
                text: "Settings"
                onClicked: drawer.open()
            }
        }

        MouseArea {
            id: mouseAreaForMenuLayout
            anchors.fill: menuLayout
            hoverEnabled: true
            acceptedButtons: Qt.NoButton
        }

        Drawer {
            id: drawer
            parent: parent
            width: 320
            height: window.height
            edge: Qt.RightEdge
            dragMargin: 0 // disable drag-opening

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16

                Text {
                    Layout.fillWidth: true
                    font.pointSize: 14
                    font.bold: true
                    text: qsTr("3D viewer settings")
                }

                Item {
                    height: 16
                }

                Text {
                    text: qsTr("Point size")
                }

                Slider {
                    id: pointSizeSlider
                    Layout.fillWidth: true
                    value: 1.5
                    from: 1
                    to: 5
                    stepSize: 0.1
                }

                Text {
                    text: qsTr("Ambient component")
                }

                Slider {
                    id: ambientSlider
                    Layout.fillWidth: true
                    value: 0.5
                    from: 0
                    to: 1
                }

                Text {
                    text: qsTr("Diffuse component")
                }

                Slider {
                    id: diffuseSlider
                    Layout.fillWidth: true
                    value: 0.7
                    from: 0
                    to: 1
                }

                Text {
                    text: qsTr("Specular component")
                }

                Slider {
                    id: specularSlider
                    Layout.fillWidth: true
                    value: 0.1
                    from: 0
                    to: 1
                }

                Text {
                    text: qsTr("Specular power")
                }

                Slider {
                    id: shininessSlider
                    Layout.fillWidth: true
                    value: 25
                    from: 0
                    to: 200
                }

                Item {
                    Layout.fillHeight: true
                }
            }
        }
    }

    ColumnLayout {
        anchors.centerIn: parent
        visible: !pointCloudReader.pointCloud
        width: Math.min(600, Math.max(400, window.width/2))
        height: Math.min(600, Math.max(400, window.height/2))

        spacing: 24

        Rectangle {
            Layout.fillWidth: true
            radius: 3
            border.width: 1
            border.color: "#22000000"
            clip: true
            color: "#22ffffff"
            height: 48

            Text {
                anchors.fill: parent
                anchors.margins: 10
                text: qsTr("Load a point cloud ...")
                font.pointSize: 14
                color: "#333"
                verticalAlignment: Text.AlignVCenter
            }

            MouseArea {
                anchors.fill: parent
                onClicked: fileDialog.open()
            }
        }

        Text {
            Layout.fillWidth: true
            font.pointSize: 14
            font.italic: true
            color: "#666"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            text: qsTr("or open one of the recently used files")
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: 3
            border.width: 1
            border.color: "#22000000"
            clip: true
            color: "#22ffffff"

            ListView {
                id: recentFilesListView
                anchors.fill: parent
                model: recentFiles

                ScrollBar.vertical: ScrollBar {
                    active: true
                }

                delegate: Item {
                    id: delegate
                    width: recentFilesListView.width
                    height: 56

                    property string fileUrl: modelData
                    property string fileName: fileUrl.slice(1 + fileUrl.lastIndexOf("/"))
                    property string filePath: fileUrl.slice(7)

                    property color textColor: "#333"

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 2

                        Text {
                            Layout.fillWidth: true
                            text: delegate.fileName
                            font.pointSize: 14
                            color: textColor
                        }

                        Text {
                            text: delegate.filePath
                            font.pointSize: 11
                            color: Qt.lighter(textColor, 2)
                        }
                    }

                    Rectangle {
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                        height: 1
                        color: "#11000000"
                    }

                    MouseArea {
                        id: mouseArea
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: {
                            pointCloudReader.source = modelData
                            lastOpenedFile = modelData
                        }
                    }
                }
            }
        }
    }
}
