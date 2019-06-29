import Qt3D.Render 2.12

Camera {
    id: mainCamera
    projectionType: CameraLens.PerspectiveProjection
    fieldOfView: 20
    nearPlane: 0.01
    farPlane: 1000.0
    viewCenter: Qt.vector3d(0.0, 0.0, 0.0)
    upVector: Qt.vector3d(0.0, 1.0, 0.0)
    position: Qt.vector3d(0.0, 0.0, 300.0)
}
