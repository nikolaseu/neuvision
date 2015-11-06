#include "zincompletecirclegridpatternfinderplugin.h"

#include "zincompletecirclegridpatternfinder.h"

namespace Z3D {

QString ZIncompleteCircleGridPatternFinderPlugin::name()
{
    return QString(metaObject()->className());
}

QString ZIncompleteCircleGridPatternFinderPlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCalibrationPatternFinder::Ptr> ZIncompleteCircleGridPatternFinderPlugin::getPatternFinders()
{
    return QList<ZCalibrationPatternFinder::Ptr>() << ZCalibrationPatternFinder::Ptr(new ZIncompleteCircleGridPatternFinder());
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zincompletecirclegridcalibrationpatternfinderplugin, Z3D::ZIncompleteCircleGridPatternFinderPlugin)
#endif
