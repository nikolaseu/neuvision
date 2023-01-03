import Qt3D.Core
import Qt3D.Extras
import Qt3D.Render

import Z3D.ZPointCloud

Entity {
    id: root

    property int levelOfDetail: -1
    property alias levelOfDetailCamera: levelOfDetail.camera
    property alias pointCloud: pointCloudGeometry.pointCloud
    property Transform transform: Transform {}

    property real pointSize: 2
    property real splatSize: pointSize * (levelOfDetail.enabled ? Math.pow(1.5, pointCloudGeometry.levelOfDetail) : 1)
    property vector3d lightPosition: Qt.vector3d(0.0, 0.0, 100.0)
    property vector3d lightIntensity: Qt.vector3d(1.0, 1.0, 1.0)
    property real ambient: 0.5
    property real diffuse: 0.7
    property real specular: 0.1
    property real shininess: 25.0

    property bool renderPointsAsDiscs: false
    property bool showColors: true
    property color defaultColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)

    GeometryRenderer {
        id: pointCloudMesh

        primitiveType: pointCloud.hasTriangles
                       ? GeometryRenderer.Triangles
                       : GeometryRenderer.Points

        geometry: PointCloudGeometry {
            id: pointCloudGeometry
            levelOfDetail: root.levelOfDetail < 0
                           ? levelOfDetail.currentIndex
                           : root.levelOfDetail
        }
    }

//    PhongMaterial {
//        id: material
//        ambient: Qt.lighter(root.defaultColor, root.ambient)
//        diffuse: Qt.lighter(root.defaultColor, 2 * root.diffuse)
//        specular: Qt.lighter(root.defaultColor, 2 * root.specular)
//        shininess: root.shininess
//    }

    MetalRoughMaterial {
        id: material
        baseColor: defaultColor
        metalness: specular
        roughness: shininess / 50
    }

    PerVertexColorMaterial {
        id: perVertexColorMaterial
    }

    ShaderProgram {
        id: pointCloudShaderProgram
        vertexShaderCode: loadSource("qrc:/z3d.neuvision/Z3D/ZPointCloud/shaders/pointcloud.vert")
        fragmentShaderCode: loadSource("qrc:/z3d.neuvision/Z3D/ZPointCloud/shaders/pointcloud.frag")
        geometryShaderCode: loadSource("qrc:/z3d.neuvision/Z3D/ZPointCloud/shaders/pointcloud.geom")
    }

    ShaderProgram {
        id: pointCloudShaderProgramSimple
        vertexShaderCode: loadSource("qrc:/z3d.neuvision/Z3D/ZPointCloud/shaders/pointcloud_basic.vert")
        fragmentShaderCode: loadSource("qrc:/z3d.neuvision/Z3D/ZPointCloud/shaders/pointcloud_basic.frag")
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
                Parameter { name: "defaultColor"; value: Qt.vector3d(root.defaultColor.r, root.defaultColor.g, root.defaultColor.b) },
                Parameter { name: "hasColors"; value: root.pointCloud.hasColors && root.showColors },
                Parameter { name: "hasNormals"; value: root.pointCloud.hasNormals },
                Parameter { name: "hasRadii"; value: root.pointCloud.hasRadii }
            ]

            techniques: Technique {
                filterKeys: [
                    FilterKey { name: "renderingStyle"; value: "forward" }
                ]

                graphicsApiFilter {
                    api: GraphicsApiFilter.RHI
                    profile: GraphicsApiFilter.NoProfile
                    majorVersion: 1
                    minorVersion: 0
                }

                renderPasses: [
                    RenderPass {
                        shaderProgram: pointCloud.hasNormals && root.renderPointsAsDiscs
                                       ? pointCloudShaderProgram
                                       : pointCloudShaderProgramSimple
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
        !pointCloud.hasTriangles
            ? pointCloudMaterial
            : (pointCloud.hasColors && root.showColors)
                ? perVertexColorMaterial
                : material,
        transform,
        levelOfDetail
    ]
}
