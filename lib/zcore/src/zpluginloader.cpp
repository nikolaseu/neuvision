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
#elif defined(Q_OS_ANDROID)
        if (pluginsDir.dirName() == "files") {
            pluginsDir.cdUp();
            folder = "lib";
        }
#endif

        if (!pluginsDir.cd(folder)) {
            qWarning() << "plugins folder" << folder << "not found in" << pluginsDir.absolutePath()
                       << "available dirs/files:" << pluginsDir.entryList(QDir::AllEntries);
            return;
        }
    }

    qDebug() << "searching for plugin folders in" << pluginsDir.absolutePath();

    QStringList filters;
#if defined(Q_OS_WIN)
    filters << "*.dll";
#elif defined(Q_OS_MAC)
    filters << "*.dylib";
#elif defined(Q_OS_ANDROID)
    filters << "libplugins_*.so";
#else
    filters << "*.so";
#endif

#if defined(Q_OS_ANDROID)
    auto foldersList = QStringList() << ".";
#else
    auto foldersList = pluginsDir.entryList(QDir::AllDirs | QDir::NoDot | QDir::NoDotDot);
#endif
    auto folderCount = foldersList.size();
    auto folderIndex = -1;
    for (const auto &folderName : foldersList) {
        folderIndex++;
        QString pluginsFolderName = pluginsDir.absoluteFilePath(folderName);
        qDebug() << "loading plugins from" << pluginsFolderName;
        QDir currentPluginsDir(pluginsFolderName);
        currentPluginsDir.setNameFilters(filters);

        auto fileList = currentPluginsDir.entryList(QDir::Files);
        auto fileCount = fileList.size();
        auto fileIndex = -1;
        for (const auto &fileName : fileList) {
            fileIndex++;
            QString pluginFileName = currentPluginsDir.absoluteFilePath(fileName);

            float progress = (float(folderIndex) + float(fileIndex)/fileCount) / folderCount;
            emit progressChanged(progress, tr("Loading %1").arg(fileName));

            QPluginLoader loader(pluginFileName);
            QObject *pluginInstance = loader.instance();
            ZCorePlugin *plugin = qobject_cast<ZCorePlugin *>(pluginInstance);
            if (plugin) {
                qDebug() << "plugin:" << pluginFileName << "\n"
                         << "id:" << plugin->id() << "\n"
                         << "version:" << plugin->version() << "\n"
                         << "metaData:" << loader.metaData();
#if defined(Q_OS_ANDROID)
                QString pluginType = fileName.section('_', 1, 1);
                m_plugins[pluginType] << plugin;
#else
                m_plugins[folderName] << plugin;
#endif
            } else {
                qWarning() << "error loading plugin" << fileName << "->" << loader.errorString();
            }
        }
    }

    emit progressChanged(1.f, tr("Finished loading plugins"));
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
