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

#include "zpluginloader.h"

#include "zcoreplugin.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QJsonObject>

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
    QDir pluginsDir = QFileInfo(QCoreApplication::applicationFilePath()).absoluteDir();

    /// if no folder is indicated, use standard search path, i.e. "plugins"
    if (folder.isEmpty()) {
        folder = "plugins";
    }

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
            qWarning() << "plugins folder" << folder << "not found in" << pluginsDir.absolutePath()
                       << "available dirs/files:" << pluginsDir.entryList(QDir::AllEntries);
            return;
        }
    }
#endif

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

            ZCorePlugin *plugin = new ZCorePlugin(pluginFileName);
            if (plugin->load()) {
                qDebug() << "\nplugin:" << pluginFileName
                         << "\nid:" << plugin->id()
                         << "\nversion:" << plugin->version()
                         << "\nmetaData:" << plugin->metaData();
#if defined(Q_OS_ANDROID)
                QString pluginType = fileName.section('_', 1, 1);
                m_plugins[pluginType] << plugin;
#else
                m_plugins[folderName] << plugin;
#endif
            } else {
                delete plugin;
                qWarning() << "error loading plugin" << fileName << "->" << plugin->errorString();
            }
        }
    }

    emit progressChanged(1.f, tr("Finished loading plugins"));
}

void ZPluginLoader::unloadPlugins()
{

}

const QList<ZCorePlugin *> ZPluginLoader::plugins(const QString &pluginType)
{
    auto &plugins = instance()->m_plugins;
    if (plugins.find(pluginType) != plugins.end()) {
        return plugins.at(pluginType);
    }

    return QList<ZCorePlugin *>();
}

ZPluginLoader::ZPluginLoader(QObject *parent)
    : QObject(parent)
{

}

} // namespace Z3D
