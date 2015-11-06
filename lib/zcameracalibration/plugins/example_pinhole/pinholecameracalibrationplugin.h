#ifndef Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATIONPLUGIN_H
#define Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATIONPLUGIN_H

#include <QObject>
#include <QVariantMap>

#include "cameracalibrationplugininterface.h"

namespace Z3D
{

class PinholeCameraCalibrationPlugin : public QObject, CameraCalibrationPluginInterface
{
    Q_OBJECT

#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameracalibration.cameracalibrationplugininterface" FILE "pinhole.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::CameraCalibrationPluginInterface)

public:
    /// plugin information
    virtual QString name();
    virtual QString version();

    /// calibration utilities
    virtual CameraCalibrationInterface::Ptr getCalibration(QVariantMap options);
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATIONPLUGIN_H
