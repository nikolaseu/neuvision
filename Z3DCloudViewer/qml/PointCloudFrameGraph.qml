import Qt3D.Render 2.0

TechniqueFilter {
    id: root

    // Expose camera to allow user to choose which camera to use for rendering
    property alias camera: cameraSelector.camera
    property alias window: surfaceSelector.surface

    // Expose the layers we'll manage during rendering
    readonly property Layer backgroundLayer: Layer {}
    readonly property Layer pointsLayer: Layer {}

    // Select the forward rendering Technique of any used Effect
    matchAll: [ FilterKey { name: "renderingStyle"; value: "forward" } ]

    RenderSurfaceSelector {
        id: surfaceSelector

        // Use the whole viewport
        Viewport {
            id: viewport
            normalizedRect: Qt.rect(0.0, 0.0, 1.0, 1.0)

            ClearBuffers {
                buffers: ClearBuffers.ColorDepthBuffer
                LayerFilter { layers: root.backgroundLayer }
            }

            Viewport {
                id: mainViewport
                normalizedRect: Qt.rect(0.0, 0.0, 1.0, 1.0)

                // Use the specified camera
                CameraSelector {
                    id: cameraSelector

                    LayerFilter {
                        layers: root.pointsLayer
                        RenderStateSet {
                            renderStates: [
                                PointSize { sizeMode: PointSize.Programmable }, //supported since OpenGL 3.2
                                DepthTest { depthFunction: DepthTest.Less }
                            ]
                        }
                    }
                }
            }
        }
    }
}
