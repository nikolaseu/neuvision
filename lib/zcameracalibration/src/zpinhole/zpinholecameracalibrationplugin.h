#ifndef Z3D_CAMERACALIBRATION___ZPINHOLECAMERACALIBRATIONPLUGIN_H
#define Z3D_CAMERACALIBRATION___ZPINHOLECAMERACALIBRATIONPLUGIN_H

#include <QObject>
#include <QVariantMap>

#include "zcameracalibrationplugininterface.h"

namespace Z3D
{

class ZPinholeCameraCalibrationPlugin : public ZCameraCalibrationPluginInterface
{
    Q_OBJECT
/*
#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameracalibration.cameracalibrationplugininterface" FILE "pinhole.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::ZCameraCalibrationPluginInterface)
*/

    // ZCorePlugin interface
public:
    virtual QString id() override;
    virtual QString name() override;
    virtual QString version() override;

    // ZCameraCalibrationPluginInterface interface
public:
    virtual ZCameraCalibration::Ptr getCalibration(QVariantMap options) override;
    virtual QList<ZCameraCalibrator *> getCameraCalibrators() override;
    virtual QList<ZMultiCameraCalibrator *> getMultiCameraCalibrators() override;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATION___ZPINHOLECAMERACALIBRATIONPLUGIN_H
