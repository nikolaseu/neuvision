#ifndef Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPLUGININTERFACE_H
#define Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPLUGININTERFACE_H

#include "zcameracalibration_global.h"
#include "zcameracalibration.h"
#include "zcameracalibrator.h"
#include "zmulticameracalibrator.h"

#include "zcoreplugin.h"

namespace Z3D
{

class Z3D_CAMERACALIBRATION_SHARED_EXPORT ZCameraCalibrationPluginInterface : public ZCorePlugin
{
public:
    virtual ~ZCameraCalibrationPluginInterface() {}

    /// calibration utilities
    virtual ZCameraCalibration::Ptr getCalibration(QVariantMap options) = 0;

    virtual QList<ZCameraCalibrator *> getCameraCalibrators() = 0;

    virtual QList<ZMultiCameraCalibrator *> getMultiCameraCalibrators() = 0;
};

} // namespace Z3D

#define ZCameraCalibrationPluginInterface_iid "z3d.zcameracalibration.zcameracalibrationplugininterface"

Q_DECLARE_INTERFACE(Z3D::ZCameraCalibrationPluginInterface, ZCameraCalibrationPluginInterface_iid)

#endif // Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPLUGININTERFACE_H
