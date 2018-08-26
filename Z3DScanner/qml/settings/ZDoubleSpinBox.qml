import QtQuick 2.9
import QtQuick.Controls 2.2

Item {
    id: root

    implicitHeight: spinbox.implicitHeight
    implicitWidth: spinbox.implicitWidth

    property real value: 0
    property real from: 0
    property real to: 1
    property real stepSize: 0.01
    property int decimals: 2

    property real _scale: Math.pow(10, decimals)

    SpinBox {
        id: spinbox

        from: root.from * _scale
        value: root.value * _scale
        to: root.to * _scale
        stepSize: root.stepSize * _scale

        onValueChanged: {
            root.value = value / _scale
        }

        validator: DoubleValidator {
            bottom: Math.min(spinbox.from, spinbox.to)
            top:  Math.max(spinbox.from, spinbox.to)
        }

        textFromValue: function(value, locale) {
            return Number(value / _scale).toLocaleString(locale, 'f', root.decimals)
        }

        valueFromText: function(text, locale) {
            return Number.fromLocaleString(locale, text) * _scale
        }
    }
}
