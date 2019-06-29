import Qt3D.Core 2.12
import Qt3D.Render 2.12
import Qt3D.Extras 2.12

import Z3D.ZPointCloud 1.0

Entity {
    id: root

    property alias pointCloud: pointCloudGeometry.pointCloud
    property Transform transform: Transform {}

    property real pointSize: 2
    property vector3d lightPosition: Qt.vector3d(0.0, 0.0, 100.0)
    property vector3d lightIntensity: Qt.vector3d(1.0, 1.0, 1.0)
    property real ambient: 0.5
    property real diffuse: 0.7
    property real specular: 0.1
    property real shininess: 25.0

    property bool showColors: false
    property color defaultColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)

    property bool hasNormals: false

    GeometryRenderer {
        id: pointCloudMesh

        primitiveType: GeometryRenderer.Points
        geometry: PointCloudGeometry {
            id: pointCloudGeometry
        }
    }

    Material {
        id: material

        effect: Effect {
            parameters: [
                Parameter { name: "ka"; value: Qt.vector3d(root.ambient, root.ambient, root.ambient) },
                Parameter { name: "kd"; value: Qt.vector3d(root.diffuse, root.diffuse, root.diffuse) },
                Parameter { name: "ks"; value: Qt.vector3d(root.specular, root.specular, root.specular) },
                Parameter { name: "shininess"; value: root.shininess },
                Parameter { name: "lightPosition";  value: root.lightPosition },
                Parameter { name: "lightIntensity"; value: root.lightIntensity },
                Parameter { name: "splatSize"; value: root.pointSize },
                Parameter { name: "hasColors"; value: root.showColors },
                Parameter { name: "defaultColor"; value: Qt.vector3d(root.defaultColor.r, root.defaultColor.g, root.defaultColor.b) },
                Parameter { name: "hasNormals"; value: root.hasNormals }
            ]

            techniques: Technique {
                filterKeys: [
                    FilterKey { name: "renderingStyle"; value: "forward" }
                ]

                graphicsApiFilter {
                    api: GraphicsApiFilter.OpenGL
                    profile: GraphicsApiFilter.CoreProfile
                    majorVersion: 3
                    minorVersion: 2
                }

                renderPasses: [
                    RenderPass {
                        shaderProgram: ShaderProgram {
                            id: pointCloudShaderProgram
                            vertexShaderCode: loadSource("qrc:/Z3D/ZPointCloud/shaders/pointcloud.vert")
                            fragmentShaderCode: loadSource("qrc:/Z3D/ZPointCloud/shaders/pointcloud.frag")
                            geometryShaderCode: loadSource("qrc:/Z3D/ZPointCloud/shaders/pointcloud.geom")
                        }
                        renderStates: [
                            DepthTest { depthFunction: DepthTest.Less },
                            CullFace { mode: CullFace.NoCulling }
                        ]
                    }
                ]
            }
        }
    }

    components: [
        pointCloudMesh,
        material,
        transform
    ]
}
