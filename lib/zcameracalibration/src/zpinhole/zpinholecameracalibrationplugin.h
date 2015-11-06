#ifndef Z3D_CAMERACALIBRATION___ZPINHOLECAMERACALIBRATIONPLUGIN_H
#define Z3D_CAMERACALIBRATION___ZPINHOLECAMERACALIBRATIONPLUGIN_H

#include <QObject>
#include <QVariantMap>

#include "zcameracalibrationplugininterface.h"

namespace Z3D
{

class ZPinholeCameraCalibrationPlugin : public QObject, public ZCameraCalibrationPluginInterface
{
    Q_OBJECT
/*
#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameracalibration.cameracalibrationplugininterface" FILE "pinhole.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::ZCameraCalibrationPluginInterface)
*/
public:
    /// plugin information
    virtual QString name();
    virtual QString version();

    /// calibration utilities
    virtual ZCameraCalibration::Ptr getCalibration(QVariantMap options);
    virtual QList<ZCameraCalibrator *> getCameraCalibrators();
    virtual QList<ZMultiCameraCalibrator *> getMultiCameraCalibrators();
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATION___ZPINHOLECAMERACALIBRATIONPLUGIN_H
