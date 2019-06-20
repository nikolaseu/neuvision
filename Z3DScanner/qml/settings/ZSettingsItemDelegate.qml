import QtQuick 2.10
import QtQuick.Controls 2.3
import QtQuick.Layouts 1.3

import Z3D.ZSettingsItem 1.0

Item {
    id: root
    implicitHeight: loader.implicitHeight
    implicitWidth: 200
    enabled: setting.writable

    property QtObject setting

    Component {
        id: boolSetting

        CheckDelegate {
            text: setting.label
            checked: setting.value
            onCheckedChanged: setting.value = checked
        }
    }

    Component {
        id: intSetting

        CheckDelegate {
            id: control

            text: setting.label

            indicator: SpinBox {
                x: control.mirrored ? control.leftPadding : control.width - width - control.rightPadding
                y: control.topPadding + (control.availableHeight - height) / 2

                value: setting.value
                from: setting.minimum
                to: setting.maximum
                onValueChanged: setting.value = value
            }
        }
    }

    Component {
        id: floatSetting

        CheckDelegate {
            id: control

            text: setting.label

            indicator: ZDoubleSpinBox {
                x: control.mirrored ? control.leftPadding : control.width - width - control.rightPadding
                y: control.topPadding + (control.availableHeight - height) / 2

                value: setting.value
                from: setting.minimum
                to: setting.maximum
                onValueChanged: setting.value = value
            }
        }
    }

    Component {
        id: enumSetting

        CheckDelegate {
            id: control

            text: setting.label

            indicator: ComboBox {
                x: control.mirrored ? control.leftPadding : control.width - width - control.rightPadding
                y: control.topPadding + (control.availableHeight - height) / 2

                model: setting.options
                currentIndex: setting.value
                onCurrentIndexChanged: setting.value = currentIndex
            }
        }
    }

    Component {
        id: commandSetting

        ItemDelegate {
            id: control
            text: setting.label
            onClicked: setting.execute()
        }
    }

    Component {
        id: unknownSetting

        ItemDelegate {
            text: qsTr("%1: %2").arg(setting.label).arg(setting.value)
        }
    }

    Component {
        id: unreadableSetting

        ItemDelegate {
            text: qsTr("%1 [Non-readable]").arg(setting.label)
        }
    }

    Loader {
        id: loader
        width: parent.width
    }

    state: "unknown_setting" // default
    states: [
        State {
            name: "unknown_setting"
            PropertyChanges { target: loader; sourceComponent: unknownSetting }
        },
        State {
            name: "unreadable_setting"
            when: !setting.readable
            PropertyChanges { target: loader; sourceComponent: unreadableSetting }
        },
        State {
            name: "bool_setting"
            when: setting.type === ZSettingsItem.SettingsItemTypeBool
            PropertyChanges { target: loader; sourceComponent: boolSetting }
        },
        State {
            name: "int_setting"
            when: setting.type === ZSettingsItem.SettingsItemTypeInt
            PropertyChanges { target: loader; sourceComponent: intSetting }
        },
        State {
            name: "float_setting"
            when: setting.type === ZSettingsItem.SettingsItemTypeFloat
            PropertyChanges { target: loader; sourceComponent: floatSetting }
        },
        State {
            name: "enum_setting"
            when: setting.type === ZSettingsItem.SettingsItemTypeEnum
            PropertyChanges { target: loader; sourceComponent: enumSetting }
        },
        State {
            name: "command_setting"
            when: setting.type === ZSettingsItem.SettingsItemTypeCommand
            PropertyChanges { target: loader; sourceComponent: commandSetting }
        }
    ]
}
