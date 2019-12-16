import QtQuick 2.10
import QtQuick.Controls 2.3

ListView {
    id: root
    clip: true

    section.property: "path"
    section.delegate: Pane {
        width: root.width
        height: sectionLabel.implicitHeight + 20

        Label {
            id: sectionLabel
            text: section
            anchors.centerIn: parent
        }
    }

    delegate: ZSettingsItemDelegate {
        width: root.width
        setting: qtObject
    }

    ScrollIndicator.vertical: ScrollIndicator { }
}
