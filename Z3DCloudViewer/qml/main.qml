import Qt3D.Core 2.0 as Q3D
import Qt3D.Extras 2.0
import Qt3D.Input 2.0
import Qt3D.Render 2.0

import QtQuick 2.10
import QtQuick.Controls 2.3
import QtQuick.Dialogs 1.3
import QtQuick.Layouts 1.3
import QtQuick.Scene3D 2.0

import Z3D.PointCloud 1.0

ApplicationWindow {
    id: window
    title: qsTr("Point cloud viewer")
    width: 1200
    height: 800
    visible: true

    FileDialog {
        id: fileDialog
        title: qsTr("Open a point cloud")
        folder: shortcuts.home
        nameFilters: ["PLY files (*.ply)", "STL files (*.stl)", "PCD files (*.pcd)"]
        onAccepted: {
            pointCloudReader.source = fileDialog.fileUrl
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
                        activeFrameGraph: PointCloudFrameGraph {
                            id: renderer
                            camera: mainCamera
                        }
                    },
                    InputSettings {
                        eventSource: window
                        enabled: true
                    }
                ]

                BackgroundEntity {
                    id: background
                    layer: renderSettings.activeFrameGraph.backgroundLayer
                    colorTop: Qt.rgba(0.4, 0.4, 0.4, 1)
                    colorBottom: Qt.rgba(0.8, 0.8, 0.8, 1)
                }

                PointCloudEntity {
                    id: pointCloud
                    layer: renderSettings.activeFrameGraph.pointsLayer
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
                    value: 2
                    from: 1
                    to: 4
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

    Button {
        anchors.centerIn: parent
        visible: !pointCloudReader.pointCloud
        padding: 20
        text: qsTr("Open a point cloud ...")
        onClicked: fileDialog.open()
    }
}
