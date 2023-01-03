//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "ZCore/zpluginloader.h"

#include "ZCore/zcoreplugin.h"
#include "ZCore/zlogging.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcore", QtDebugMsg)

namespace Z3D
{

ZPluginLoader *ZPluginLoader::m_instance = nullptr; //! FIXME avoid static/singletons!

ZPluginLoader *ZPluginLoader::instance()
{
    if (!m_instance) {
        m_instance = new ZPluginLoader();
    }

    return m_instance;
}

void ZPluginLoader::loadPlugins(const QString &folder1)
{
    QDir pluginsDir = QFileInfo(QCoreApplication::applicationFilePath()).absoluteDir();

    /// if no folder is indicated, use standard search path, i.e. "plugins"
    const QString folder = (!folder1.isEmpty())
                                   ? folder1
                                   : "plugins";

#if !defined(Q_OS_ANDROID)
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
            zWarning() << "plugins folder" << folder << "not found in" << pluginsDir.absolutePath()
                       << "available dirs/files:" << pluginsDir.entryList(QDir::AllEntries);
            return;
        }
    }
#endif

    zDebug() << "searching for plugin folders in" << pluginsDir.absolutePath();

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
    const auto foldersList = QStringList() << ".";
#else
    const auto foldersList = pluginsDir.entryList(QDir::AllDirs | QDir::NoDot | QDir::NoDotDot);
#endif
    const auto folderCount = foldersList.size();
    auto folderIndex = -1;
    for (const auto &folderName : foldersList) {
        folderIndex++;
        const QString pluginsFolderName = pluginsDir.absoluteFilePath(folderName);
        zDebug() << "loading plugins from" << pluginsFolderName;
        const QDir currentPluginsDir(pluginsFolderName);

        const auto fileList = currentPluginsDir.entryList(filters, QDir::Files);
        const auto fileCount = fileList.size();
        auto fileIndex = -1;
        for (const auto &fileName : fileList) {
            fileIndex++;
            const QString pluginFileName = currentPluginsDir.absoluteFilePath(fileName);

            ZCorePlugin *const plugin = new ZCorePlugin(pluginFileName);

            const float progress = (float(folderIndex) + float(fileIndex)/fileCount) / folderCount;
            emit progressChanged(progress, tr("Loading %1").arg(fileName));

            zDebug().nospace().noquote()
                    << "found plugin: " << pluginFileName
                    << ", metaData: " << QJsonDocument(plugin->metaData()).toJson();

#if defined(Q_OS_ANDROID)
            QString pluginType = fileName.section('_', 1, 1);
            m_plugins[pluginType] << plugin;
#else
            m_plugins[folderName] << plugin;
#endif
        }
    }

    emit progressChanged(1.f, tr("Finished loading plugins"));
}

void ZPluginLoader::unloadPlugins()
{

}

QList<ZCorePlugin *> ZPluginLoader::plugins(const QString &pluginType)
{
    const auto &plugins = instance()->m_plugins;
    if (plugins.contains(pluginType)) {
        return plugins.at(pluginType);
    }

    return {};
}

ZPluginLoader::ZPluginLoader(QObject *parent)
    : QObject(parent)
{

}

} // namespace Z3D
