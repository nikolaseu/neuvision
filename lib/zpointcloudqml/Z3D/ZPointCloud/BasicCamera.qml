import Qt3D.Render

import QtQuick

Camera {
    id: mainCamera
    projectionType: CameraLens.PerspectiveProjection
    fieldOfView: 20
    nearPlane: 0.01
    farPlane: 1000.0
    viewCenter: Qt.vector3d(0.0, 0.0, 0.0)
    upVector: Qt.vector3d(0.0, 1.0, 0.0)
    position: Qt.vector3d(0.0, 0.0, 300.0)

    property real animationDuration: 500
    property bool animationEnabled: true
    // we cannot use something different because it's restarted as camera can move continuously
    property int animationEasing: Easing.InOutQuad
    Behavior on position {
        enabled: mainCamera.animationEnabled
        Vector3dAnimation {
            duration: mainCamera.animationDuration
            easing.type: mainCamera.animationEasing
        }
    }
    Behavior on viewCenter {
        enabled: mainCamera.animationEnabled
        Vector3dAnimation {
            duration: mainCamera.animationDuration
            easing.type: mainCamera.animationEasing
        }
    }
    Behavior on upVector {
        enabled: mainCamera.animationEnabled
        Vector3dAnimation {
            duration: mainCamera.animationDuration
            easing.type: mainCamera.animationEasing
        }
    }

    function zoomTo(point) {
        mainCamera.viewCenter = point
    }
}
