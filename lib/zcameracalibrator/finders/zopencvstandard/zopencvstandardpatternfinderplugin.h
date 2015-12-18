#ifndef Z3D_ZOPENCVSTANDARDPATTERNFINDERPLUGIN_H
#define Z3D_ZOPENCVSTANDARDPATTERNFINDERPLUGIN_H

#include <QObject>

#include "zcalibrationpatternfinderplugininterface.h"

namespace Z3D {

class ZOpenCVStandardPatternFinderPlugin : public ZCalibrationPatternFinderPluginInterface
{
    Q_OBJECT

#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameracalibrator.calibrationpatternfinderplugininterface" FILE "zopencvstandard.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::ZCalibrationPatternFinderPluginInterface)

    // ZCorePlugin interface
public:
    virtual QString id() override;
    virtual QString name() override;
    virtual QString version() override;

    // ZCalibrationPatternFinderPluginInterface interface
public:
    virtual QList<ZCalibrationPatternFinder::Ptr> getPatternFinders() override;
};

} // namespace Z3D

#endif // Z3D_ZOPENCVSTANDARDPATTERNFINDERPLUGIN_H
