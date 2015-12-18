#include "zcalibrationpatternfinderprovider.h"

#include "zcalibrationpatternfinderplugininterface.h"
#include "zpluginloader.h"

#include <QDebug>

namespace Z3D
{

QMap< QString, ZCalibrationPatternFinderPluginInterface *> ZCalibrationPatternFinderProvider::m_plugins;

void ZCalibrationPatternFinderProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("calibrationpatternfinder");

    for (auto pluginInstance : list) {
        auto *plugin = qobject_cast<ZCalibrationPatternFinderPluginInterface *>(pluginInstance);
        if (plugin) {
            qDebug() << "pattern finder plugin loaded. type:" << plugin->id()
                     << "version:" << plugin->version();
            m_plugins.insert(plugin->id(), plugin);
        } else {
            qWarning() << "invalid pattern finder plugin:" << plugin;
        }
    }
}

void ZCalibrationPatternFinderProvider::unloadPlugins()
{
    foreach(ZCalibrationPatternFinderPluginInterface *plugin, m_plugins.values())
        delete plugin;

    m_plugins.clear();
}

QList<ZCalibrationPatternFinder::Ptr> ZCalibrationPatternFinderProvider::getAll()
{
    QList<ZCalibrationPatternFinder::Ptr> finderList;

    /// load different calibration pattern finder types
    foreach (ZCalibrationPatternFinderPluginInterface *plugin, m_plugins)
        finderList << plugin->getPatternFinders();

    return finderList;
}

ZCalibrationPatternFinderProvider::ZCalibrationPatternFinderProvider()
{

}

} // namespace Z3D
