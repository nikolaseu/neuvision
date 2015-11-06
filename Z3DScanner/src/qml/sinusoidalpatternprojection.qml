import QtQuick 2.5

Rectangle {
    id: root
    width: 480
    height: 320
    color: "#888"

    property real size: 2000
    property bool isFullscreen: true

    /*ShaderEffectSource {
        id: binaryTexSource
        sourceItem: Image { source: "qrc:///pattern/binary.bmp" }
        hideSource: true
        visible: config.threePhasePattern ? false : (config.binaryPattern)
    }

    ShaderEffectSource {
        id: grayTexSource
        sourceItem: Image { source: "qrc:///pattern/gray.bmp" }
        hideSource: true
        visible: config.threePhasePattern ? false : (!config.binaryPattern)
    }*/

    ShaderEffect {
        id: shader
        width: 1024*2
        height: 1024*2
        anchors.centerIn: parent
        //anchors.fill: parent

        NumberAnimation on phase {
            from: 0
            to: 2 * Math.PI
            loops: Animation.Infinite
            duration: 3000
        }

        property real intensity: 1
        property real frequency: 64 * (2 * Math.PI)
        property real phase:     0
        property bool vertical:  false
        property bool inverted:  false

        /*
        property real intensity: config.intensity
        property real frequency: config.frequency * (config.threePhasePattern ? 2 * Math.PI   : 1)
        property real phase:     config.phase     * (config.threePhasePattern ? Math.PI / 180 : 2)
        property bool vertical:  config.vertical
        property bool inverted:  config.inverted
        */

        property string threePhaseShader:
            "uniform highp float intensity;" +
            "uniform highp float frequency;" +
            "uniform highp float phase;" +
            "uniform highp bool vertical;" +
            "varying highp vec2 qt_TexCoord0;" +
            "void main() {" +
            "    highp float p;" +
            "    if (vertical) {" +
            "        p = (0.5 + 0.5 * cos(phase + frequency * qt_TexCoord0.y));" +
            "    } else {" +
            "        p = (0.5 + 0.5 * cos(phase + frequency * qt_TexCoord0.x));" +
            "    }" +
            "    p = p * intensity;" +
            "    gl_FragColor = vec4(p, p, p, 1);" +
            "}"

        /*
        property variant binaryTex: binaryTexSource
        property string binaryShader:
            "uniform highp float width;" +
            "uniform highp float height;" +
            "uniform highp float intensity;" +
            "uniform highp float frequency;" +
            "uniform highp float phase;" +
            "uniform highp bool vertical;" +
            "uniform highp bool inverted;" +
            "uniform sampler2D binaryTex;" +
            "varying highp vec2 qt_TexCoord0;" +
            "void main() {" +
            "    highp vec4 cv;" +
            "    if (vertical) {" +
            "        cv = texture2D(binaryTex, vec2((frequency/11.+0.05), mod((phase + qt_TexCoord0.y*height)/1024., 1.)));" +
            "    } else {" +
            "        cv = texture2D(binaryTex, vec2((frequency/11.+0.05), mod(-(phase + qt_TexCoord0.x*width)/1024., 1.)));" +
            "    }" +
            "    highp float c;" +
            "    if (inverted) {c = 1. - cv[0];} else {c = cv[0];}" +
            "    c = c * intensity;" +
            "    gl_FragColor = vec4(c, c, c, 1);" +
            "}"

        property variant grayTex: grayTexSource
        property string grayBinaryShader:
            "uniform highp float width;" +
            "uniform highp float height;" +
            "uniform highp float intensity;" +
            "uniform highp float frequency;" +
            "uniform highp float phase;" +
            "uniform highp bool vertical;" +
            "uniform highp bool inverted;" +
            "uniform sampler2D grayTex;" +
            "varying highp vec2 qt_TexCoord0;" +
            "void main() {" +
            "    highp vec4 cv;" +
            "    if (vertical) {" +
            "        cv = texture2D(grayTex, vec2((frequency/11.+0.05), mod((phase + qt_TexCoord0.y*height)/1024., 1.)));" +
            "    } else {" +
            "        cv = texture2D(grayTex, vec2((frequency/11.+0.05), mod(-(phase + qt_TexCoord0.x*width)/1024., 1.)));" +
            "    }" +
            "    highp float c;" +
            "    if (inverted) {c = 1. - cv[0];} else {c = cv[0];}" +
            "    c = c * intensity;" +
            "    gl_FragColor = vec4(c, c, c, 1);" +
            "}"
        */

        //fragmentShader: config.threePhasePattern ? threePhaseShader : (config.binaryPattern ? binaryShader : grayBinaryShader)
        fragmentShader: threePhaseShader
    }

    Rectangle {
        anchors.fill: parent
        color: "white"
        opacity: 0.7
        visible: !root.isFullscreen

        Column {
            anchors.centerIn: parent

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "Drag this window to projector"
                font.pixelSize: 16
                font.bold: true
            }

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "and click here to maximize"
                font.pixelSize: 16
                font.bold: true
            }
        }
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent

        onClicked: {
            if (!root.isFullscreen) {
                window.showFullScreen();
            } else {
                window.showNormal();
            }
            root.isFullscreen = !root.isFullscreen;
        }
    }
}
