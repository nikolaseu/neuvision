import Qt3D.Render 2.12

import QtQuick 2.12

Camera {
    id: mainCamera
    projectionType: CameraLens.PerspectiveProjection
    fieldOfView: 20
    nearPlane: 0.01
    farPlane: 1000.0
    viewCenter: Qt.vector3d(0.0, 0.0, 0.0)
    upVector: Qt.vector3d(0.0, 1.0, 0.0)
    position: Qt.vector3d(0.0, 0.0, 300.0)

    ParallelAnimation {
        id: zoomAnimation
        property int duration: 200
        Vector3dAnimation {
            id: viewCenterAnimation
            target: mainCamera
            property: "viewCenter"
            duration: zoomAnimation.duration
        }
        Vector3dAnimation {
            id: positionAnimation
            target: mainCamera
            property: "position"
            duration: zoomAnimation.duration
        }
    }

    function zoomTo(point) {
        viewCenterAnimation.from = mainCamera.viewCenter
        viewCenterAnimation.to = point

        const d = 0.2 * (mainCamera.viewCenter.minus(mainCamera.position)).length();
        positionAnimation.from = mainCamera.position
        mainCamera.viewCenter = point
        mainCamera.translate(Qt.vector3d(0, 0, d), Camera.DontTranslateViewCenter)
        positionAnimation.to = mainCamera.position

        zoomAnimation.running = true
    }
}
