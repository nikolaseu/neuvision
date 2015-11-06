#ifndef Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDERPROVIDER_H
#define Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDERPROVIDER_H

#include "zcameracalibrator_global.h"
#include "zcalibrationpatternfinder.h"

#include <QMap>
#include <QSettings>
#include <QVariantMap>

namespace Z3D
{

class ZCalibrationPatternFinderPluginInterface;

class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZCalibrationPatternFinderProvider
{

public:
    static void loadPlugins(QString folder = QString());
    static void unloadPlugins();

    static QList<ZCalibrationPatternFinder::Ptr> getAll();

private:
    explicit ZCalibrationPatternFinderProvider();

    static QMap< QString, ZCalibrationPatternFinderPluginInterface *> m_plugins;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDERPROVIDER_H
