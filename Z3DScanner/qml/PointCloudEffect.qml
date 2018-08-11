import Qt3D.Render 2.0

Effect {
    parameters: [
        Parameter { name: "lightPosition";  value: root.lightPosition },
        Parameter { name: "lightIntensity"; value: root.lightIntensity },
        Parameter { name: "pointSize"; value: root.pointSize }
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

        renderPasses: RenderPass {
            shaderProgram: ShaderProgram {
                id: pointCloudShaderProgram
                vertexShaderCode: loadSource("qrc:/shaders/pointcloud.vert")
                fragmentShaderCode: loadSource("qrc:/shaders/pointcloud.frag")
            }
            renderStates: [
//                DepthTest { depthFunction: DepthTest.Less }
                DepthTest { depthFunction: DepthTest.Always }
            ]
        }
    }
}
