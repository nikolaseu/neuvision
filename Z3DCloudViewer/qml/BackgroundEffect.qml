import Qt3D.Core 2.0
import Qt3D.Render 2.0

Effect {
    id: root

    techniques: [
        Technique {
            graphicsApiFilter {
                api: GraphicsApiFilter.OpenGL
                profile: GraphicsApiFilter.CoreProfile
                majorVersion: 3
                minorVersion: 1
            }

            filterKeys: [ FilterKey { name: "renderingStyle"; value: "forward" } ]

            renderPasses: [
                RenderPass {
                    shaderProgram: ShaderProgram {
                        vertexShaderCode:   loadSource( "qrc:/shaders/background.vert" )
                        fragmentShaderCode: loadSource( "qrc:/shaders/background.frag" )
                    }

                    renderStates: [
                        DepthTest { depthFunction: DepthTest.Always }
                    ]
                }
            ]
        }
    ]
}
