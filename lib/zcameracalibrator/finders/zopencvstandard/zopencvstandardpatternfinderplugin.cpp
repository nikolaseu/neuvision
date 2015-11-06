#include "zopencvstandardpatternfinderplugin.h"

#include "zchessboardcalibrationpatternfinder.h"
#include "zcirclegridcalibrationpatternfinder.h"

namespace Z3D {

QString ZOpenCVStandardPatternFinderPlugin::name()
{
    return QString(metaObject()->className());
}

QString ZOpenCVStandardPatternFinderPlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCalibrationPatternFinder::Ptr> ZOpenCVStandardPatternFinderPlugin::getPatternFinders()
{
    QList<ZCalibrationPatternFinder::Ptr> finderList;

    finderList << ZCalibrationPatternFinder::Ptr(new ZCircleGridCalibrationPatternFinder());
    finderList << ZCalibrationPatternFinder::Ptr(new ZChessboardCalibrationPatternFinder());

    return finderList;
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zopencvstandardcalibrationpatternfinderplugin, Z3D::ZOpenCVStandardPatternFinderPlugin)
#endif
