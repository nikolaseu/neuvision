import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0

import Z3D.PointCloud 1.0

Entity {
    id: root

    property alias pointCloud: pointCloudGeometry.pointCloud
    property Layer layer: null
    property Transform transform: Transform {}

    property real pointSize: 2
    property vector3d lightPosition: Qt.vector3d(0.0, 0.0, 100.0)
    property vector3d lightIntensity: Qt.vector3d(1.0, 1.0, 1.0)
    property real ambient: 0.5
    property real diffuse: 0.7
    property real specular: 0.1
    property real shininess: 25.0

    GeometryRenderer {
        id: pointCloudMesh

        primitiveType: GeometryRenderer.Points
        geometry: PointCloudGeometry {
            id: pointCloudGeometry
        }
    }

    Material {
        id: material

        effect: PointCloudEffect {
            parameters: [
                Parameter { name: "ka"; value: Qt.vector3d(root.ambient, root.ambient, root.ambient) },
                Parameter { name: "kd"; value: Qt.vector3d(root.diffuse, root.diffuse, root.diffuse) },
                Parameter { name: "ks"; value: Qt.vector3d(root.specular, root.specular, root.specular) },
                Parameter { name: "shininess"; value: shininess },
                Parameter { name: "lightPosition";  value: root.lightPosition },
                Parameter { name: "lightIntensity"; value: root.lightIntensity },
                Parameter { name: "pointSize"; value: root.pointSize }
            ]
        }
    }

    components: [
        layer,
        pointCloudMesh,
        material,
        transform
    ]
}
