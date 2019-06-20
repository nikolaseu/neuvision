import QtQuick 2.5

Rectangle {
    id: root
    width: 480
    height: 320
    color: "black"

    property real size: 2000

    Item {
        anchors.fill: parent
        opacity: config.intensity
        visible: !config.vertical

        // BINARY PATTERNS
        Image {
            id: shader
            width: (sourceSize.width*2) * root.size
            height: sourceSize.height
            x: -(config.currentPattern*4+1) * root.size
            //anchors.verticalCenter: parent.verticalCenter
            source: config.inverted
                    ? (config.useGrayBinary ? "qrc:///zstructuredlight/pattern/gray.2048.inv.bmp" : "qrc:///zstructuredlight/pattern/binary.2048.inv.bmp")
                    : (config.useGrayBinary ? "qrc:///zstructuredlight/pattern/gray.2048.bmp"     : "qrc:///zstructuredlight/pattern/binary.2048.bmp")
        }
    }

    Item {
        anchors.fill: parent
        opacity: config.intensity
        visible: config.vertical
        rotation: 90
        transformOrigin: Item.TopLeft

        // BINARY PATTERNS
        Image {
            id: shader2
            width: (sourceSize.width*2) * root.size
            height: sourceSize.height
            x: -(config.currentPattern*4+1) * root.size
            y: -height
            //anchors.verticalCenter: parent.verticalCenter
            source: config.inverted
                    ? (config.useGrayBinary ? "qrc:///zstructuredlight/pattern/gray.2048.inv.bmp" : "qrc:///zstructuredlight/pattern/binary.2048.inv.bmp")
                    : (config.useGrayBinary ? "qrc:///zstructuredlight/pattern/gray.2048.bmp"     : "qrc:///zstructuredlight/pattern/binary.2048.bmp")
        }
    }
}
