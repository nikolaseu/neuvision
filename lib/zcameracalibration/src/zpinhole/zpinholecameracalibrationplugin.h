#pragma once

#include <QObject>
#include <QVariantMap>

#include "zcameracalibrationplugininterface.h"

namespace Z3D
{

class ZPinholeCameraCalibrationPlugin : public ZCameraCalibrationPluginInterface
{
    Q_OBJECT

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
