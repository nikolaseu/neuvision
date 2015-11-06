#include "zcalibrationpatternfinderprovider.h"

#include "zcalibrationpatternfinderplugininterface.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QPluginLoader>

namespace Z3D
{

QMap< QString, ZCalibrationPatternFinderPluginInterface *> ZCalibrationPatternFinderProvider::m_plugins;

void ZCalibrationPatternFinderProvider::loadPlugins(QString folder)
{
    QDir pluginsDir = QDir(folder);

    /// if no folder is indicated, use current running folder and standard search path
    if (folder.isEmpty()) {
        pluginsDir = QCoreApplication::applicationDirPath();

    #if defined(Q_OS_WIN)
//        if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
//            pluginsDir.cdUp();
    #elif defined(Q_OS_MAC)
        if (pluginsDir.dirName() == "MacOS") {
            pluginsDir.cdUp();
            pluginsDir.cdUp();
            pluginsDir.cdUp();
        }
    #endif

        if (!pluginsDir.cd("plugins/calibrationpatternfinder")) {
            qWarning() << "standard calibration pattern finder plugin folder 'plugins/calibrationpatternfinder' not found";
            return;
        }
    }

    qDebug() << "searching for calibration pattern finder plugins in" << pluginsDir.currentPath();

    QStringList filters;
    filters << "*.dll" << "*.so";
    pluginsDir.setNameFilters(filters);

    foreach (QString fileName, pluginsDir.entryList(QDir::Files)) {
        QPluginLoader loader(pluginsDir.absoluteFilePath(fileName));
        QObject *pluginInstance = loader.instance();
        if (pluginInstance) {
            ZCalibrationPatternFinderPluginInterface *plugin = qobject_cast<ZCalibrationPatternFinderPluginInterface *>(pluginInstance);
            if (plugin) {
                qDebug() << "calibration pattern finder plugin loaded:" << plugin->name()
                         << "version:" << plugin->version()
                         << "filename:" << fileName;

                m_plugins.insert(plugin->name(), plugin);
            } else {
                qWarning() << "invalid calibration pattern finder plugin:" << fileName;
            }
        } else {
            qWarning() << "error loading calibration pattern finder plugin" << fileName << "->" << loader.errorString();
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
