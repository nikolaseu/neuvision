import Qt3D.Core 2.0 as Q3D
import Qt3D.Extras 2.0
import Qt3D.Input 2.0
import Qt3D.Render 2.0

import QtQuick 2.10
import QtQuick.Controls 2.3
import QtQuick.Layouts 1.3
import QtQuick.Scene3D 2.0

import Z3D.ZPointCloud 1.0
import "settings"

ApplicationWindow {
    id: window
    title: qsTr("Z3D Scanner")
    width: 1200
    height: 800
    visible: true

    color: Qt.platform.os == "osx" ? "transparent" : "#ddd"

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

//                    pointSize: pointSizeSlider.value
//                    lightPosition: mainCamera.position
//                    ambient: ambientSlider.value
//                    diffuse: diffuseSlider.value
//                    specular: specularSlider.value
//                    shininess: shininessSlider.value

                    showColors: true

                    transform: Q3D.Transform {
                        // center in 0,0,0
                        translation: Qt.vector3d(-pointCloud.pointCloud.center.x, -pointCloud.pointCloud.center.y, -pointCloud.pointCloud.center.z)
                    }
                }
            }
        }

        RowLayout {
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.margins: 4
            opacity: mouseArea.containsMouse ? 1 : 0.3

            Behavior on opacity {
                OpacityAnimator {
                    duration: 100
                }
            }

            Button {
                text: qsTr("Scan")
                onClicked: scanner.scan()
            }

            Button {
                text: qsTr("Settings")
                onClicked: drawer.open()
            }

            MouseArea {
                id: mouseArea
                anchors.fill: parent
                hoverEnabled: true
                acceptedButtons: Qt.NoButton
            }
        }

        Drawer {
            id: drawer
            parent: parent
            width: 320
            height: window.height
            edge: Qt.RightEdge
            dragMargin: 0 // disable drag-opening

            onClosed: {
                if (stackView.depth > 1) {
                    // go back to initial item
                    stackView.clear(StackView.Immediate)
                    stackView.push(mainStackViewMenu, StackView.Immediate)
                }
            }

            StackView {
                id: stackView
                anchors.fill: parent
                clip: true

                initialItem: mainStackViewMenu
            }

            Component {
                id: mainStackViewMenu

                Item {
                    ColumnLayout {
                        anchors.fill: parent

                        ItemDelegate {
                            Layout.fillWidth: true
                            font.pointSize: 14
                            text: qsTr("Pattern settings")
                            onClicked: {
                                stackView.push(patternProjectionSettings);
                            }
                        }

                        ItemDelegate {
                            Layout.fillWidth: true
                            font.pointSize: 14
                            text: qsTr("Reconstruction settings")
                            onClicked: {
                                stackView.push(structuredLightSystemSettings);
                            }
                        }

                        ItemDelegate {
                            Layout.fillWidth: true
                            font.pointSize: 14
                            text: qsTr("3D viewer settings")
                            onClicked: stackView.push(settingPage)
                        }

                        Item {
                            Layout.fillHeight: true
                        }
                    }
                }
            }

            Component {
                id: patternProjectionSettings

                Item {
                    ColumnLayout {
                        anchors.fill: parent

                        RowLayout {
                            Button {
                                Layout.maximumWidth: height
                                text: "<"
                                onClicked: stackView.pop()
                            }

                            Label {
                                Layout.fillWidth: true
                                font.bold: true
                                text: qsTr("Pattern projetion settings")
                            }
                        }

                        ZSettingsListView {
                            id: patternProjectionSettingsListView
                            Layout.fillHeight: true
                            Layout.fillWidth: true

                            model: scanner.patternProjectionSettings
                        }
                    }
                }
            }

            Component {
                id: structuredLightSystemSettings

                Item {
                    ColumnLayout {
                        anchors.fill: parent

                        RowLayout {
                            Button {
                                Layout.maximumWidth: height
                                text: "<"
                                onClicked: stackView.pop()
                            }

                            Label {
                                Layout.fillWidth: true
                                font.bold: true
                                text: qsTr("Structured light system settings")
                            }
                        }

                        ZSettingsListView {
                            id: structuredLightSystemSettingsListView
                            Layout.fillHeight: true
                            Layout.fillWidth: true

                            model: scanner.structuredLightSystemSettings
                        }
                    }
                }
            }

            Component {
                id: settingPage

                Item {
                    ColumnLayout {
                        anchors.fill: parent

                        RowLayout {
                            Button {
                                Layout.maximumWidth: height
                                text: "<"
                                onClicked: stackView.pop()
                            }

                            Label {
                                Layout.fillWidth: true
                                font.bold: true
                                text: qsTr("3D viewer settings")
                            }
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
        }
    }

    Connections {
        target: scanner
        onCloudChanged: {
            pointCloud.pointCloud = cloud;
        }
    }
}
