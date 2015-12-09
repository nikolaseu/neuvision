#include "zcameracalibrationprovider.h"

#include "zcameracalibration.h"
#include "zcameracalibrationplugininterface.h"
#include "zpinhole/zpinholecameracalibrationplugin.h"

#include <QDebug>
#include <QDir>
#include <QPluginLoader>

namespace Z3D
{

QMap< QString, ZCameraCalibrationPluginInterface *> ZCameraCalibrationProvider::m_plugins;

void ZCameraCalibrationProvider::loadPlugins(QString folder)
{
    /// add pinhole camera calibration plugin
    ZCameraCalibrationPluginInterface *pinholePlugin = new ZPinholeCameraCalibrationPlugin;
    m_plugins.insert(pinholePlugin->name(), pinholePlugin);

    QDir pluginsDir = QDir(folder);

    /// if no folder is indicated, use current running folder and standard search path
    if (folder.isEmpty()) {
        pluginsDir = QDir::current();

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

        if (!pluginsDir.cd("plugins/cameracalibration")) {
            qWarning() << "standard camera calibration plugin folder 'plugins/cameracalibration' not found";
            return;
        }
    }

    qDebug() << "searching for camera calibration plugins in" << pluginsDir.currentPath();

    QStringList filters;
    filters << "*.dll" << "*.so";
    pluginsDir.setNameFilters(filters);

    foreach (QString fileName, pluginsDir.entryList(QDir::Files)) {
        QPluginLoader loader(pluginsDir.absoluteFilePath(fileName));
        QObject *plugin = loader.instance();
        if (plugin) {
            ZCameraCalibrationPluginInterface *calibrationPlugin = qobject_cast<ZCameraCalibrationPluginInterface *>(plugin);
            if (calibrationPlugin) {
                qDebug() << "camera calibration plugin loaded:" << calibrationPlugin->name()
                         << "version:" << calibrationPlugin->version()
                         << "filename:" << fileName;

                m_plugins.insert(calibrationPlugin->name(), calibrationPlugin);
            } else {
                qWarning() << "invalid camera calibration plugin:" << fileName;
            }
        } else {
            qWarning() << "error loading camera calibration plugin" << fileName << "->" << loader.errorString();
        }
    }
}

void ZCameraCalibrationProvider::unloadPlugins()
{
    foreach(ZCameraCalibrationPluginInterface *plugin, m_plugins.values())
        delete plugin;

    m_plugins.clear();
}

ZCameraCalibration::Ptr ZCameraCalibrationProvider::getCalibration(QString pluginName, QVariantMap options)
{
    ZCameraCalibration::Ptr calibration;

    if (m_plugins.contains(pluginName)) {
        calibration = m_plugins[pluginName]->getCalibration(options);
    } else {
        qWarning() << "camera calibration plugin not found:" << pluginName;
    }

    return calibration;
}

ZCameraCalibration::Ptr ZCameraCalibrationProvider::getCalibration(QSettings *settings)
{
    QVariantMap options;
    foreach(QString key, settings->allKeys()) {
        options[key] = settings->value(key);
    }

    QString pluginName = options["Type"].toString();

    return getCalibration(pluginName, options);
}

QList<ZCameraCalibrator *> ZCameraCalibrationProvider::getCameraCalibrators()
{
    QList<ZCameraCalibrator *> list;

    foreach(ZCameraCalibrationPluginInterface *plugin, m_plugins.values())
        list << plugin->getCameraCalibrators();

    return list;
}

QList<ZMultiCameraCalibrator *> ZCameraCalibrationProvider::getMultiCameraCalibrators()
{
    QList<ZMultiCameraCalibrator *> list;

    foreach(ZCameraCalibrationPluginInterface *plugin, m_plugins.values())
        list << plugin->getMultiCameraCalibrators();

    return list;
}

} // namespace Z3D
