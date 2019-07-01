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

import Z3D.ZPointCloud 1.0 as ZPointCloud
import Z3D.ZPointCloudViewer 1.0

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

    ZPointCloudViewerController {
        id: controller

        onFileLoaded: {
            // loading might fail, so only update if it was sucessfully opened
            window.lastOpenedFile = fileUrl;
        }

        onMessage: {
            statusBar.showMessage(message);
        }
    }

    Settings {
        id: recentFilesSettings
        category: "Recent"

        property alias recentFiles: window.recentFiles
    }

    FileDialog {
        id: fileDialog
        title: qsTr("Open a point cloud or project")
        folder: shortcuts.home
        nameFilters: ["PLY files (*.ply)", "STL files (*.stl)", "PCD files (*.pcd)", "Meshlab project (*.mlp)" ]
        onAccepted: {
            controller.loadFile(fileDialog.fileUrl);
        }
    }

    Item {
        anchors.fill: parent

        Keys.enabled: true
        Keys.onReleased: {
            if (event.key === Qt.Key_F) {
                screenRayCaster.triggerForMousePosition();
            }
        }

        Scene3D {
            id: scene3d
            anchors.fill: parent

            aspects: ["input", "logic"]
            cameraAspectRatioMode: Scene3D.AutomaticAspectRatio
            focus: true
            multisample: false // faster
            hoverEnabled: true

            Q3D.Entity {
                id: sceneRoot

                ZPointCloud.BasicCamera {
                    id: mainCamera
                    aspectRatio: scene3d.width/scene3d.height
                }

                ZPointCloud.DefaultCameraController {
                    camera: mainCamera
                }

                function printHits(desc, hits) {
                    console.log(desc, hits.length)
                    for (var i=0; i<hits.length; i++) {
                        console.log("  " + hits[i].entity.objectName, hits[i].distance,
                                    hits[i].worldIntersection.x, hits[i].worldIntersection.y, hits[i].worldIntersection.z)
                    }
                }

                components: [
                    RenderSettings {
                        id: renderSettings
                        activeFrameGraph: ForwardRenderer {
                            id: renderer
                            camera: mainCamera
                            clearColor: window.color
                        }
                        renderPolicy: RenderSettings.OnDemand
                        pickingSettings.pickMethod: PickingSettings.PrimitivePicking
                        pickingSettings.pickResultMode: PickingSettings.NearestPick
                    },
                    InputSettings {
                        eventSource: window
                        enabled: true
                    },
                    ScreenRayCaster {
                        id: screenRayCaster
                        onHitsChanged: {
//                            sceneRoot.printHits("Screen hits", hits)
                            if (hits.length > 0) {
//                                console.log("  " + hits[0].worldIntersection.x, hits[0].worldIntersection.y, hits[0].worldIntersection.z);
                                mainCamera.zoomTo(Qt.vector3d(hits[0].worldIntersection.x, hits[0].worldIntersection.y, hits[0].worldIntersection.z));
                            }
                        }

                        function triggerForMousePosition() {
                            screenRayCaster.trigger(mousePosition)
                        }

                        property var mousePosition
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
                        onPositionChanged: {
//                            console.log("mouse position changed", mouse)
                            screenRayCaster.mousePosition = Qt.point(mouse.x, mouse.y);
                        }
//                        onDoubleClicked: {
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
                    model: controller.model

                    delegate: ZPointCloud.PointCloudEntity {
                        id: pointCloudEntity

                        enabled: qtObject.visible
                        pointCloud: qtObject.pointCloud
                        levelOfDetailCamera: mainCamera

                        pointSize: pointSizeSlider.value
                        lightPosition: mainCamera.position
                        ambient: ambientSlider.value
                        diffuse: diffuseSlider.value
                        specular: specularSlider.value
                        shininess: shininessSlider.value

                        showColors: showColorsSwitch.checked
                        defaultColor: colorChooser.color

                        // if it's set to a non-negative value, it will be fixed and will not be updated automatically
//                        levelOfDetail: 4
//                        levelOfDetail: 0

                        transform: Q3D.Transform {
                            matrix: qtObject.transformation
                        }
                    }

                    onCountChanged: {
                        // center camera when something is added/removed
                        mainCamera.viewAll();
                    }
                }
            }
        }

        ColumnLayout {
            id: menuLayout
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            width: 320
            //opacity: mouseAreaForMenuLayout.containsMouse ? 1 : 0.3
            visible: pointCloudEntityInstatiator.count > 0

            Behavior on opacity {
                OpacityAnimator {
                    duration: 100
                }
            }

            RowLayout {
                Layout.fillWidth: true

                Button {
                    text: "Open file..."
                    onClicked: fileDialog.open()
                }

                Button {
                    text: "Settings"
                    onClicked: drawer.open()
                }
            }

            ListView {
                Layout.fillHeight: true
                Layout.fillWidth: true

                model: controller.model

                delegate: CheckDelegate {
                    text: qtObject.name
                    checkable: true
                    checked: qtObject.visible

                    onCheckedChanged: {
                        qtObject.visible = checked
                    }
                }
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

                SwitchDelegate {
                    id: showColorsSwitch
                    Layout.fillWidth: true
                    checked: true
                    text: qsTr("Show object colors")
                }

                CheckDelegate {
                    id: colorChooser
                    Layout.fillWidth: true
                    text: qsTr("Color")
                    enabled: !showColorsSwitch.checked
                    checkable: false

                    property color color: "#666"

                    onClicked: {
                        colorDialog.color = color
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
                            colorChooser.color = colorDialog.color;
                        }
                    }
                }

                Text {
                    text: qsTr("Point size")
                }

                Slider {
                    id: pointSizeSlider
                    Layout.fillWidth: true
                    value: 3
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

    MouseArea {
        id: mouseAreaForAvoidingProblemsWithMacTitlebar
        visible: Qt.platform.os === "osx"
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 24
    }

    ColumnLayout {
        anchors.centerIn: parent
        visible: pointCloudEntityInstatiator.count < 1
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
                            controller.loadFile(modelData)
                        }
                    }
                }
            }
        }
    }

    Rectangle {
        id: statusBar
        opacity: 0
        anchors.bottom: parent.bottom
        anchors.right: parent.right
        anchors.left: parent.left
        height: statusBarLayout.height
        color: "#66ffffff"

        function showMessage(message) {
            statusBarText.text = message;
            if (showMessageAnimation.running) {
                showMessageAnimation.restart();
            } else {
                showMessageAnimation.running = true;
            }
        }

        SequentialAnimation {
            id: showMessageAnimation
            NumberAnimation {
                target: statusBar
                property: "opacity"
                duration: 0
                from: 1
                to: 1
            }
            NumberAnimation {
                duration: 5000
            }
            NumberAnimation {
                target: statusBar
                property: "opacity"
                duration: 1000
                from: 1
                to: 0
                easing.type: Easing.InOutQuad
            }
        }

        RowLayout {
            id: statusBarLayout
            Label {
                id: statusBarText
                Layout.margins: 10
                height: implicitHeight
            }
        }
    }
}
