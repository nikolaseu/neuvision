#include "zpluginloader.h"

#include "zcoreplugin.h"

#include <QDebug>
#include <QDir>
#include <QPluginLoader>

namespace Z3D
{

ZPluginLoader *ZPluginLoader::m_instance = nullptr;

ZPluginLoader *ZPluginLoader::instance()
{
    if (!m_instance) {
        m_instance = new ZPluginLoader();
    }

    return m_instance;
}

void ZPluginLoader::loadPlugins(QString folder)
{
    QDir pluginsDir = QDir::current();

    /// if no folder is indicated, use standard search path, i.e. "plugins"
    if (folder.isEmpty()) {
        folder = "plugins";
    }

    if (!pluginsDir.cd(folder)) {
        /// if folder does not exists and we're on Win or OSX try going up, i.e. "cd .."
#if defined(Q_OS_WIN)
        if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
            pluginsDir.cdUp();
#elif defined(Q_OS_MAC)
        if (pluginsDir.dirName() == "MacOS") {
            pluginsDir.cdUp();
            pluginsDir.cdUp();
            pluginsDir.cdUp();
        }
#endif

        if (!pluginsDir.cd(folder)) {
            qWarning() << "plugins folder" << folder << "not found in" << pluginsDir.absolutePath();
            return;
        }
    }

    qDebug() << "searching for plugin folders in" << pluginsDir.absolutePath();

    QStringList filters;
#if defined(Q_OS_WIN)
    filters << "*.dll";
#elif defined(Q_OS_MAC)
    filters << "*.dylib";
#else
    filters << "*.so";
#endif

    QStringList foldersList = pluginsDir.entryList(QDir::AllDirs | QDir::NoDot | QDir::NoDotDot);

    foreach (QString folderName, foldersList) {
        QString pluginsFolderName = pluginsDir.absoluteFilePath(folderName);
        qDebug() << "loading plugins from" << pluginsFolderName;
        QDir currentPluginsDir(pluginsFolderName);
        currentPluginsDir.setNameFilters(filters);

        QList<QObject *> pluginsList;

        foreach (QString fileName, currentPluginsDir.entryList(QDir::Files)) {
            QString pluginFileName = currentPluginsDir.absoluteFilePath(fileName);
            QPluginLoader loader(pluginFileName);
            QObject *pluginInstance = loader.instance();
            ZCorePlugin *plugin = qobject_cast<ZCorePlugin *>(pluginInstance);
            if (plugin) {
                qDebug() << "plugin:" << pluginFileName << "\n"
                         << "id:" << plugin->id() << "\n"
                         << "version:" << plugin->version() << "\n"
                         << "metaData:" << loader.metaData();
                pluginsList << plugin;
            } else {
                qWarning() << "error loading plugin" << fileName << "->" << loader.errorString();
            }
        }

        auto &plugins = instance()->m_plugins;
        plugins[folderName] = pluginsList;
    }
}

void ZPluginLoader::unloadPlugins()
{

}

const QList<QObject *> ZPluginLoader::plugins(const QString &pluginType)
{
    auto &plugins = instance()->m_plugins;
    if (plugins.find(pluginType) != plugins.end()) {
        return plugins.at(pluginType);
    }

    return QList<QObject *>();
}

ZPluginLoader::ZPluginLoader(QObject *parent)
    : QObject(parent)
{

}

} // namespace Z3D
