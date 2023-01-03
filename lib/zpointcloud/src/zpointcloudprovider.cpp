//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include "ZPointCloud/zpointcloudprovider.h"

#include "ZPointCloud/zpointcloud.h"
#include "ZPointCloud/zpointcloudplugininterface.h"

#include <ZCore/zcoreplugin.h>
#include <ZCore/zlogging.h>
#include <ZCore/zpluginloader.h>

#include <QtCore/QJsonArray>
#include <QtQml/QQmlEngine>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zpointcloud", QtInfoMsg)

namespace Z3D
{

std::unordered_map<QString, std::vector<ZCorePlugin *>> ZPointCloudProvider::m_pluginLoaders;

void ZPointCloudProvider::loadPlugins()
{
    const auto list = ZPluginLoader::plugins("pointcloud");

    for (const auto &pluginLoader : list) {
        const auto id = pluginLoader->id();
        const QJsonObject metaData = pluginLoader->metaData();
        if (!metaData.contains("extensions")) {
            zWarning() << "invalid plugin, missing `extensions` field" << metaData;
            continue;
        }

        const QJsonArray extensions = metaData["extensions"].toArray();
        for (const QJsonValue &extensionValue : extensions) {
            if (!extensionValue.isString()) {
                zWarning() << "invalid plugin, unhandled `extensions` item:" << extensionValue;
                continue;
            }
            const QString ext = extensionValue.toString().toLower(); /// use lowercase for id!
            zDebug().nospace() << "registering plugin for extension: " << ext << " [" << id << "]";
            m_pluginLoaders[ext].push_back(pluginLoader);
        }
    }
}

void ZPointCloudProvider::unloadPlugins()
{
    for (const auto &[_, pluginLoaders] : m_pluginLoaders) {
        for (const auto &pluginLoader : pluginLoaders) {
            pluginLoader->unload();
        }
    }

    m_pluginLoaders.clear();
}

std::vector<QString> ZPointCloudProvider::formats()
{
    std::vector<QString> v;
    for (const auto &[ext, _] : m_pluginLoaders) {
        v.push_back(ext);
    }
    return v;
}

ZPointCloudUniquePtr ZPointCloudProvider::loadPointCloud(const QString &fileName)
{
    zDebug() << "Trying to read PointCloud from" << fileName;

    const QFileInfo fileInfo(fileName);
    if (!fileInfo.exists()) {
        zWarning() << "Invalid file:" << fileName;
        return nullptr;
    }

    const QString extension = fileInfo.suffix().toLower(); /// use lowercase for id!
    if (m_pluginLoaders.contains(extension)) {
        const auto &pluginLoaders = m_pluginLoaders.at(extension);
        for (const auto &pluginLoader : pluginLoaders) {
            if (auto *plugin = pluginLoader->instance<ZPointCloudPluginInterface>()) {
                if (auto pc = plugin->loadPointCloud(fileName)) {
                    return pc;
                }
            }
        }
    }

    zWarning() << "Failed to read PointCloud. No plugin can handle the format of the file:" << fileName;
    return nullptr;
}

} // namespace Z3D
