#ifndef Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDERPLUGININTERFACE_H
#define Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDERPLUGININTERFACE_H

#include "zcameracalibrator_global.h"
#include "zcalibrationpatternfinder.h"
#include "zcoreplugin.h"

namespace Z3D
{

class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZCalibrationPatternFinderPluginInterface : public ZCorePlugin
{
    Q_OBJECT

public:
    virtual ~ZCalibrationPatternFinderPluginInterface() {}

    /// calibration utilities
    virtual QList<ZCalibrationPatternFinder::Ptr> getPatternFinders() = 0;
};

} // namespace Z3D

#define ZCalibrationPatternFinderPluginInterface_iid "z3d.zcameracalibrator.zcalibrationpatternfinderplugininterface"

Q_DECLARE_INTERFACE(Z3D::ZCalibrationPatternFinderPluginInterface, ZCalibrationPatternFinderPluginInterface_iid)

#endif // Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDERPLUGININTERFACE_H
