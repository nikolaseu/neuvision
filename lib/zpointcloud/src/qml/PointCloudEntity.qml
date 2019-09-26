import Qt3D.Core 2.12
import Qt3D.Render 2.12
import Qt3D.Extras 2.12

import Z3D.ZPointCloud 1.0

Entity {
    id: root

    property int levelOfDetail: -1
    property alias levelOfDetailCamera: levelOfDetail.camera
    property alias pointCloud: pointCloudGeometry.pointCloud
    property Transform transform: Transform {}

    property real pointSize: 2
    property real splatSize: 0.5 * pointSize * (levelOfDetail.enabled ? Math.pow(1.5, pointCloudGeometry.levelOfDetail) : 1)
    property vector3d lightPosition: Qt.vector3d(0.0, 0.0, 100.0)
    property vector3d lightIntensity: Qt.vector3d(1.0, 1.0, 1.0)
    property real ambient: 0.5
    property real diffuse: 0.7
    property real specular: 0.1
    property real shininess: 25.0

    property bool renderPointsAsDiscs: true
    property bool showColors: true
    property color defaultColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)

    GeometryRenderer {
        id: pointCloudMesh

        primitiveType: pointCloudGeometry.hasTriangles
                       ? GeometryRenderer.Triangles
                       : GeometryRenderer.Points

        geometry: PointCloudGeometry {
            id: pointCloudGeometry
            levelOfDetail: root.levelOfDetail < 0
                           ? levelOfDetail.currentIndex
                           : root.levelOfDetail
        }
    }

    MetalRoughMaterial {
        id: material
        baseColor: defaultColor
        metalness: specular
        roughness: shininess
    }

    PerVertexColorMaterial {
        id: perVertexColorMaterial
    }

    ShaderProgram {
        id: pointCloudShaderProgram
        vertexShaderCode: loadSource("qrc:/Z3D/ZPointCloud/shaders/pointcloud.vert")
        fragmentShaderCode: loadSource("qrc:/Z3D/ZPointCloud/shaders/pointcloud.frag")
        geometryShaderCode: loadSource("qrc:/Z3D/ZPointCloud/shaders/pointcloud.geom")
    }

    ShaderProgram {
        id: pointCloudShaderProgramSimple
        vertexShaderCode: loadSource("qrc:/Z3D/ZPointCloud/shaders/pointcloud_basic.vert")
        fragmentShaderCode: loadSource("qrc:/Z3D/ZPointCloud/shaders/pointcloud_basic.frag")
    }

    Material {
        id: pointCloudMaterial

        effect: Effect {
            parameters: [
                Parameter { name: "ka"; value: Qt.vector3d(root.ambient, root.ambient, root.ambient) },
                Parameter { name: "kd"; value: Qt.vector3d(root.diffuse, root.diffuse, root.diffuse) },
                Parameter { name: "ks"; value: Qt.vector3d(root.specular, root.specular, root.specular) },
                Parameter { name: "shininess"; value: root.shininess },
                Parameter { name: "lightPosition";  value: root.lightPosition },
                Parameter { name: "lightIntensity"; value: root.lightIntensity },
                Parameter { name: "splatSize"; value: root.splatSize },
                Parameter { name: "pointSize"; value: root.pointSize },
                Parameter { name: "hasColors"; value: pointCloudGeometry.hasColors && root.showColors },
                Parameter { name: "defaultColor"; value: Qt.vector3d(root.defaultColor.r, root.defaultColor.g, root.defaultColor.b) },
                Parameter { name: "hasNormals"; value: pointCloudGeometry.hasNormals }
            ]

            techniques: Technique {
                filterKeys: [
                    FilterKey { name: "renderingStyle"; value: "forward" }
                ]

                graphicsApiFilter {
                    api: GraphicsApiFilter.OpenGL
                    profile: GraphicsApiFilter.CoreProfile
                    majorVersion: 3
                    minorVersion: 3
                }

                renderPasses: [
                    RenderPass {
                        shaderProgram: pointCloudGeometry.hasNormals && root.renderPointsAsDiscs ? pointCloudShaderProgram : pointCloudShaderProgramSimple
                        renderStates: [
                            PointSize { sizeMode: PointSize.Programmable },
                            DepthTest { depthFunction: DepthTest.Less },
                            CullFace { mode: CullFace.NoCulling }
                        ]
                    }
                ]
            }
        }
    }

    LevelOfDetail {
        id: levelOfDetail
        enabled: root.levelOfDetail < 0
        thresholds: [200, 300, 500, 800, 1200]
        thresholdType: LevelOfDetail.DistanceToCameraThreshold
//        thresholdType: LevelOfDetail.ProjectedScreenPixelSizeThreshold
//        volumeOverride: null // this shit is not working
    }

    components: [
        pointCloudMesh,
        !pointCloudGeometry.hasTriangles
            ? pointCloudMaterial
            : (pointCloudGeometry.hasColors && root.showColors)
                ? perVertexColorMaterial
                : material,
        transform,
        levelOfDetail
    ]
}
