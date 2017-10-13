import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0

import Z3D.PointCloud 1.0

Entity {
    id: root
    property alias pointCloud: pointCloudGeometry.pointCloud
    property Layer layer: null
    property string log: "status: " + pointCloudShaderProgram.status + "\nlog: " + pointCloudShaderProgram.log
    property real pointSize: 2

    property Transform transform : Transform {}

    property GeometryRenderer pointCloudMesh: GeometryRenderer {
        geometry: PointCloudGeometry {
            id: pointCloudGeometry
        }
        primitiveType: GeometryRenderer.Points
    }

    property Material materialPoint: PerVertexColorMaterial {
    }

    property Material materialPointCustom: Material {
        effect: Effect {
            techniques: Technique {
                graphicsApiFilter {
                    api: GraphicsApiFilter.OpenGL
                    profile: GraphicsApiFilter.CoreProfile
                    majorVersion: 3
                    minorVersion: 1
                }

                filterKeys: [
                    FilterKey { name: "renderingStyle"; value: "forward" }
                ]

                renderPasses: RenderPass {
                    shaderProgram: ShaderProgram {
                        id: pointCloudShaderProgram
                        vertexShaderCode: loadSource("qrc:/shaders/pointcloud.vert")
                        fragmentShaderCode: loadSource("qrc:/shaders/pointcloud.frag")
                    }
                    renderStates: [
                        DepthTest { depthFunction: DepthTest.Always }
                    ]
                }
            }
        }
        parameters: [
            Parameter { name: "pointSize"; value: root.pointSize }
        ]
    }

//    components: [ layer, pointCloudMesh, materialPoint, transform ]
    components: [ layer, pointCloudMesh, materialPointCustom, transform ]
}
