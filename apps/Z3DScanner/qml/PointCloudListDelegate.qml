import QtQuick 2.12
import QtQuick.Controls 2.12

Item {
    id: root

    property alias checked: checkbox.checked
    property alias checkable: checkbox.checkable
    property alias text: checkbox.text
    property alias hovered: mouseArea.containsMouse

    implicitHeight: layout.implicitHeight

    Column {
        id: layout
        width: root.width

        CheckBox {
            id: checkbox
            width: parent.width
            checkable: true
            hoverEnabled: true
        }

        Rectangle {
            width: parent.width
            height: 1
            color: "#11000000"
        }
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent

        preventStealing: true
        acceptedButtons: Qt.NoButton
        hoverEnabled: true
    }
}
