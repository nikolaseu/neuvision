#ifndef Z3D_ZINCOMPLETECIRCLEGRIDPATTERNFINDERPLUGIN_H
#define Z3D_ZINCOMPLETECIRCLEGRIDPATTERNFINDERPLUGIN_H

#include <QObject>

#include "zcalibrationpatternfinderplugininterface.h"

namespace Z3D {

class ZIncompleteCircleGridPatternFinderPlugin : public QObject, ZCalibrationPatternFinderPluginInterface
{
    Q_OBJECT

#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameracalibrator.calibrationpatternfinderplugininterface" FILE "zincompletecirclegrid.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::ZCalibrationPatternFinderPluginInterface)

public:
    virtual QString name();
    virtual QString version();
    virtual QList<ZCalibrationPatternFinder::Ptr> getPatternFinders();
};

} // namespace Z3D

#endif // Z3D_ZINCOMPLETECIRCLEGRIDPATTERNFINDERPLUGIN_H
