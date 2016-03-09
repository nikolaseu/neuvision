#pragma once

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
    static void loadPlugins();
    static void unloadPlugins();

    static QList<ZCalibrationPatternFinder::Ptr> getAll();

private:
    explicit ZCalibrationPatternFinderProvider();

    static QMap< QString, ZCalibrationPatternFinderPluginInterface *> m_plugins;
};

} // namespace Z3D
