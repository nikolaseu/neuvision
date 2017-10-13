import QtQuick 2.1 as QQ2
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0

Entity {
    id: root
    property real hue: 0.0
    property alias animateColors: hueAnim.running
    property alias color1: _private.color1
    property alias color2: _private.color2
    property Layer layer: null

    QQ2.NumberAnimation {
        id: hueAnim
        target: root
        property: "hue"
        from: 0.0; to: 1.0
        duration: 200000
        running: false
        loops: QQ2.Animation.Infinite
    }

    Entity {
        id: _private
        property color color1: Qt.hsla( (hue + 0.59) % 1, 0.53, 0.59 )
        property color color2: Qt.hsla( (hue + 0.59) % 1, 1.0, 0.15 )
    }

    components: [ layer, mesh, transform, material ]

    PlaneMesh {
        id: mesh
        width: 2.0
        height: 2.0
        meshResolution: Qt.size( 2, 2 )
    }

    Transform {
        id: transform
        // Rotate the plane so that it faces us
        rotation: fromAxisAndAngle(Qt.vector3d(1, 0, 0), 90)
    }

    Material {
        id: material
        effect: BackgroundEffect {}
        parameters: [
            Parameter { name: "color1"; value: Qt.vector3d( _private.color1.r, _private.color1.g, _private.color1.b ) },
            Parameter { name: "color2"; value: Qt.vector3d( _private.color2.r, _private.color2.g, _private.color2.b ) }
        ]
    }
}
