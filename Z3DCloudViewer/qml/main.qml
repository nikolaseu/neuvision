import Qt3D.Core 2.0 as Q3D
import Qt3D.Extras 2.0
import Qt3D.Input 2.0
import Qt3D.Render 2.0
import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Layouts 1.1
import QtQuick.Scene3D 2.0
import QtQuick.Dialogs 1.3

import Z3D.PointCloud 1.0

ApplicationWindow {
    id: window
    title: qsTr("Point cloud viewer")
    width: 1200
    height: 800
    visible: true

    menuBar: MenuBar {
        id: menuBar

        Menu {
            id: fileMenu
            title: qsTr("File")

            MenuItem {
                text: qsTr("Open file ...")
                onTriggered: fileDialog.open()
            }
        }
    }

    FileDialog {
        id: fileDialog
        title: "Open a point cloud"
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

    ColumnLayout {
        anchors.fill: parent

        Slider {
            id: pointSizeSlider
            Layout.fillWidth: true
            value: 2
            minimumValue: 1
            maximumValue: 4
        }

        Scene3D {
            id: scene3d
            Layout.fillWidth: true
            Layout.fillHeight: true

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

                OrbitCameraController {
                    camera: mainCamera
                }

                components: [
                    RenderSettings {
                        id: renderSettings
                        activeFrameGraph: PointCloudForwardRenderer {
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
                    // animateColors: true
                    color1: Qt.rgba(0.9, 0.9, 0.9, 1)
                    color2: Qt.rgba(0.8, 0.8, 0.8, 1)
                }

                PointCloudEntity {
                    id: pointCloud
                    layer: renderSettings.activeFrameGraph.pointsLayer
                    pointCloud: pointCloudReader.pointCloud
                    pointSize: pointSizeSlider.value

                    transform : Q3D.Transform {
                        // center in 0,0,0
                        translation: Qt.vector3d(-pointCloud.pointCloud.center.x, -pointCloud.pointCloud.center.y, -pointCloud.pointCloud.center.z)
                    }
                }
            }
        }
    }
}
