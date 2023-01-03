import Qt3D.Core as Q3D
import Qt3D.Extras
import Qt3D.Input
import Qt3D.Logic // for FrameAction
import Qt3D.Render

import Qt5Compat.GraphicalEffects

import QtCore
import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts
import QtQuick.Scene3D

import Z3D.ZPointCloud 1.0 as ZPointCloud
import "settings"

ApplicationWindow {
    id: window
    title: qsTr("Z3D Scanner")
    width: 1200
    height: 800
    visible: true

    color: "#ddd"

    QtObject {
        id: viewerSettings

        property real pointSize: 2

        property real ambient: 0.5
        property real diffuse: 0.7
        property real specular: 0.1
        property real shininess: 10.0

        property bool showColors: false
        property color defaultColor: Qt.rgba(0.6, 0.6, 0.6, 1)
    }

    Item {
        anchors.fill: parent

        Keys.enabled: true
        Keys.onReleased: function(event) {
            if (event.key === Qt.Key_F) {
                screenRayCaster.triggerForMousePosition();
            }
        }

        Scene3D {
            id: scene3d
            anchors.fill: parent

            aspects: ["input", "logic"]
            cameraAspectRatioMode: Scene3D.AutomaticAspectRatio
            focus: true // required for keyboard
            multisample: false // faster
            hoverEnabled: true // required for mouse handler

            Q3D.Entity {
                id: sceneRoot

                Environment {
                    /// the skybox and environment lights, for PBR
                    id: environment
                }

                ZPointCloud.BasicCamera {
                    id: mainCamera
                    aspectRatio: scene3d.width/scene3d.height
                    fieldOfView: 60
                }

                ZPointCloud.DefaultCameraController {
                    camera: mainCamera
                }

                Q3D.Entity {
                    // These lights are mostly for standard shading, non-PBR
                    components: [
                        DirectionalLight {
                            // This light is to illuminate the front faces
                            intensity: 0.5
                            worldDirection: mainCamera.viewCenter.minus(mainCamera.position).normalized()
                        },
                        DirectionalLight {
                            // This light is to illuminate the back faces
                            intensity: 0.2
                            worldDirection: mainCamera.position.minus(mainCamera.viewCenter).normalized()
                        }
                    ]
                }

                components: [
                    RenderSettings {
                        id: renderSettings
                        activeFrameGraph: RenderSurfaceSelector {
                            ClearBuffers {
                                buffers : ClearBuffers.ColorDepthBuffer
                                clearColor: window.color
                                CameraSelector {
                                    camera: mainCamera
                                    FrustumCulling {
                                        // frustum culling is per-object, so it will not really help as much as I would like
                                        RenderStateSet {
                                            renderStates: [
                                                DepthTest { depthFunction: DepthTest.Less },
                                                CullFace { mode: CullFace.NoCulling } // we want to see back faces, the inside
                                            ]
                                        }
                                    }
                                }
                            }
                        }
                        renderPolicy: RenderSettings.OnDemand // seems like this makes it stutter (at least in macOS)
                        pickingSettings.faceOrientationPickingMode: PickingSettings.FrontAndBackFace // also pick on the "inside" of objects
                        pickingSettings.pickMethod: PickingSettings.PrimitivePicking
                        pickingSettings.pickResultMode: PickingSettings.NearestPick
                    },
                    InputSettings {
                        eventSource: window
                        enabled: true
                    },
                    ScreenRayCaster {
                        id: screenRayCaster
                        onHitsChanged: function(hits) {
                            if (hits.length > 0) {
//                                console.log("  " + hits[0].worldIntersection.x, hits[0].worldIntersection.y, hits[0].worldIntersection.z);
                                mainCamera.zoomTo(Qt.vector3d(hits[0].worldIntersection.x, hits[0].worldIntersection.y, hits[0].worldIntersection.z));
                            }
                        }

                        function triggerForMousePosition() {
                            screenRayCaster.trigger(mousePosition)
                        }

                        property point mousePosition
                    },
//                    KeyboardHandler {
//                        id: keyboardHandler
//                        enabled: true
//                        focus: true
//                        sourceDevice: KeyboardDevice {}
//                        property var mouse
//                        onReleased: {
////                            if (event.key == Qt.Key_F) {
//                                screenRayCaster.trigger(Qt.point(mouse.x, mouse.y))
////                            }
//                        }
//                    },
                    MouseHandler {
                        id: mouseHandler
                        sourceDevice:  MouseDevice {}
                        onPositionChanged: function(mouse) {
//                            console.log("mouse position changed", mouse)
                            screenRayCaster.mousePosition = Qt.point(mouse.x, mouse.y);
                        }
//                        onDoubleClicked: {
////                            console.log("mouse double clicked")
//                            screenRayCaster.triggerForMousePosition();
//                        }

//                        onReleased: { screenRayCaster.trigger(Qt.point(mouse.x, mouse.y)) }
                    }
//                    ObjectPicker {
//                        id: objectPicker
//                        enabled: true

//                        onClicked: {
//                            console.log("clicked", pick)
//                            console.log("position", pick.worldIntersection)
//                        }
//                    }
                ]

                Q3D.NodeInstantiator {
                    id: pointCloudEntityInstatiator
                    model: scanner.model

                    delegate: ZPointCloud.PointCloudEntity {
                        id: pointCloudEntity

                        enabled: model.visible
                        pointCloud: model.pointCloud
                        levelOfDetailCamera: mainCamera

                        pointSize: viewerSettings.pointSize
                        lightPosition: mainCamera.position
                        ambient: viewerSettings.ambient
                        diffuse: viewerSettings.diffuse
                        specular: viewerSettings.specular
                        shininess: viewerSettings.shininess

                        showColors: !model.highlighted && viewerSettings.showColors
                        defaultColor: !model.highlighted ? viewerSettings.defaultColor : "red"

                        // if it's set to a non-negative value, it will be fixed and will not be updated automatically
                        levelOfDetail: 0

                        transform: Q3D.Transform {
                            matrix: model.transformation
                        }
                    }
                }
            }
        }

        RowLayout {
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            width: 320
            spacing: 0

            LinearGradient { //! TODO this is not efficient but it's easy to implement
                id: edge
                Layout.fillHeight: true
                width: 4
                start: Qt.point(0, 0)
                end: Qt.point(width, 0)
                property color backgroundColor: Qt.darker(window.color, 2.5)
                gradient: Gradient {
                    GradientStop { position: 0.0; color: Qt.rgba(edge.backgroundColor.r, edge.backgroundColor.g, edge.backgroundColor.b, 0.0) }
                    GradientStop { position: 0.8; color: Qt.rgba(edge.backgroundColor.r, edge.backgroundColor.g, edge.backgroundColor.b, 0.1) }
                    GradientStop { position: 1.0; color: Qt.rgba(edge.backgroundColor.r, edge.backgroundColor.g, edge.backgroundColor.b, 0.3) }
                }
            }

            Item {
                id: rightMenu
                Layout.fillHeight: true
                Layout.fillWidth: true

                ZFastBlur {
                    anchors.fill: parent
                    source: scene3d
                    radius: 64
                }

                Rectangle {
                    id: backgroundOverlay
                    anchors.fill: parent
                    property color backgroundColor: Qt.lighter(window.color, 1.1)
                    color: Qt.rgba(backgroundColor.r, backgroundColor.g, backgroundColor.b, 0.5)
                }

                ColumnLayout {
                    id: menuLayout
                    anchors.fill: parent

                    RowLayout {
                        id: headerLayout
                        Button {
                            text: "Scan"
                            onClicked: scanner.scan()
                        }

                        Button {
                            text: "View all"
                            onClicked: mainCamera.viewAll()
                        }

                        Button {
                            text: "Settings"
                            onClicked: drawer.open()
                        }
                    }

                    ListView {
                        id: pointCloudListView
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        clip: true

                        model: scanner.model

                        headerPositioning: ListView.OverlayHeader
                        header: Rectangle {
                            width: parent.width
                            height: 1
                            color: "#11000000"
                            visible: !pointCloudListView.atYBeginning
                        }

                        delegate: PointCloudListDelegate {
                            width: pointCloudListView.width
                            text: model.name
                            checked: model.visible

                            onCheckedChanged: {
                                model.visible = checked
                            }

                            onHoveredChanged: {
                                if (hovered) {
                                    pointCloudListView.currentIndex = index
                                }
                                model.highlighted = hovered && ListView.isCurrentItem
                            }

                            contextMenu: Menu {
                                MenuItem {
                                    text: "Save as..."
                                    onTriggered: {
                                        savePointCloudDialog.open();
                                    }

                                    FileDialog {
                                        id: savePointCloudDialog
                                        title: "Save as..."
                                        currentFolder: StandardPaths.standardLocations(StandardPaths.HomeLocation)[0]
                                        nameFilters: [ "PLY files (*.ply)" ]
                                        fileMode: FileDialog.SaveFile
                                        onAccepted: {
                                            ZPointCloud.Utils.savePLY(model.pointCloud, savePointCloudDialog.selectedFile);
                                        }
                                    }
                                }
                            }

                            Rectangle {
                                visible: model.highlighted
                                anchors.fill: parent
                                z: -1
                                color: backgroundOverlay.color
                            }
                        }

                        ScrollBar.vertical: ScrollBar {}
                    }
                }
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

                        Item {
                            Layout.fillHeight: true
                        }

                        SwitchDelegate {
                            id: showColorsSwitch
                            Layout.fillWidth: true
                            checked: viewerSettings.showColors
                            onCheckedChanged: {
                                viewerSettings.showColors = checked;
                            }

                            text: qsTr("Show object colors")
                        }

                        CheckDelegate {
                            id: colorChooser
                            Layout.fillWidth: true
                            text: qsTr("Color")
                            enabled: !showColorsSwitch.checked
                            checkable: false

                            property color color: viewerSettings.defaultColor

                            onClicked: {
                                colorDialog.selectedColor = color;
                                colorDialog.open();
                            }

                            indicator: Rectangle {
                                implicitWidth: 26
                                implicitHeight: 26
                                x: colorChooser.width - width - colorChooser.rightPadding
                                y: colorChooser.topPadding + colorChooser.availableHeight / 2 - height / 2
                                color: colorChooser.color
                                border.width: 1
                                border.color: Qt.darker(colorChooser.color, 1.2)
                            }

                            ColorDialog {
                                id: colorDialog
                                modality: Qt.ApplicationModal
                                onAccepted: {
                                    viewerSettings.defaultColor = colorDialog.selectedColor;
                                }
                            }
                        }

                        Text {
                            text: qsTr("Point size")
                        }

                        Slider {
                            id: pointSizeSlider
                            Layout.fillWidth: true
                            value: viewerSettings.pointSize
                            from: 1
                            to: 5
                            stepSize: 0.1

                            onValueChanged: {
                                viewerSettings.pointSize = value;
                            }
                        }

                        Text {
                            text: qsTr("Ambient component")
                        }

                        Slider {
                            id: ambientSlider
                            Layout.fillWidth: true
                            value: viewerSettings.ambient
                            from: 0.001
                            to: 1
                            onValueChanged: {
                                viewerSettings.ambient = value;
                            }
                        }

                        Text {
                            text: qsTr("Diffuse component")
                        }

                        Slider {
                            id: diffuseSlider
                            Layout.fillWidth: true
                            value: viewerSettings.diffuse
                            from: 0.001
                            to: 1
                            onValueChanged: {
                                viewerSettings.diffuse = value;
                            }
                        }

                        Text {
                            text: qsTr("Specular component")
                        }

                        Slider {
                            id: specularSlider
                            Layout.fillWidth: true
                            value: viewerSettings.specular
                            from: 0.001
                            to: 1
                            onValueChanged: {
                                viewerSettings.specular = value;
                            }
                        }

                        Text {
                            text: qsTr("Specular power")
                        }

                        Slider {
                            id: shininessSlider
                            Layout.fillWidth: true
                            value: viewerSettings.shininess
                            from: 0.001
                            to: 200
                            onValueChanged: {
                                viewerSettings.shininess = value;
                            }
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
        }
    }
}
