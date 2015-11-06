#ifndef Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPLUGININTERFACE_H
#define Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPLUGININTERFACE_H

#include "zcameracalibration_global.h"
#include "zcameracalibration.h"
#include "zcameracalibrator.h"
#include "zmulticameracalibrator.h"

#include <QtPlugin>
#include <QVariantMap>

namespace Z3D
{

class Z3D_CAMERACALIBRATION_SHARED_EXPORT ZCameraCalibrationPluginInterface
{
public:
    virtual ~ZCameraCalibrationPluginInterface() {}

    /// plugin information
    virtual QString name() = 0;
    virtual QString version() = 0;

    /// calibration utilities
    virtual ZCameraCalibration::Ptr getCalibration(QVariantMap options) = 0;

    virtual QList<ZCameraCalibrator *> getCameraCalibrators() = 0;

    virtual QList<ZMultiCameraCalibrator *> getMultiCameraCalibrators() = 0;
};

} // namespace Z3D

#define ZCameraCalibrationPluginInterface_iid "z3d.zcameracalibration.zcameracalibrationplugininterface"

Q_DECLARE_INTERFACE(Z3D::ZCameraCalibrationPluginInterface, ZCameraCalibrationPluginInterface_iid)

#endif // Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPLUGININTERFACE_H
