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

#include "zpointcloudprovider.h"

#include "zcoreplugin.h"
#include "zpluginloader.h"
#include "zpointcloud.h"
#include "zpointcloudgeometry.h"
#include "zpointcloudlistmodel.h"
#include "zpointcloudplugininterface.h"
#include "zpointcloudreader.h"

#include <QDebug>
#include <QDir>
#include <QLoggingCategory>
#include <QQmlEngine>

namespace Z3D
{

namespace // anonymous namespace
{

Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.zpointcloudprovider", QtInfoMsg)

} // anonymous namespace

QMap< QString, ZPointCloudPluginInterface *> ZPointCloudProvider::m_plugins;

void ZPointCloudProvider::registerMetaTypes()
{
    qRegisterMetaType<Z3D::ZPointCloudListModel*>("Z3D::ZPointCloudListModel*");

    qmlRegisterUncreatableType<Z3D::ZPointCloud>("Z3D.ZPointCloud", 1, 0, "PointCloud", "ZPointCloud cannot be created, must be obtained from a PointCloudReader");
    qmlRegisterType<Z3D::ZPointCloudReader>("Z3D.ZPointCloud", 1, 0, "PointCloudReader");
    qmlRegisterType<Z3D::ZPointCloudGeometry>("Z3D.ZPointCloud", 1, 0, "PointCloudGeometry");
}

void ZPointCloudProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("pointcloud");

    for (auto pluginLoader : list) {
        auto *plugin = pluginLoader->instance<ZPointCloudPluginInterface>();
        if (plugin) {
            qDebug(loggingCategory) << "pointcloud plugin loaded. type:" << pluginLoader->id();
            m_plugins.insert(pluginLoader->id(), plugin);
        } else {
            qWarning(loggingCategory) << "invalid pointcloud plugin:" << plugin;
        }
    }
}

void ZPointCloudProvider::unloadPlugins()
{
    for (const auto *plugin : m_plugins.values()) {
        delete plugin;
    }

    m_plugins.clear();
}

ZPointCloudPtr ZPointCloudProvider::loadPointCloud(const QString &fileName)
{
    qDebug(loggingCategory) << "Trying to read PointCloud from" << fileName;

    for (const auto *plugin : m_plugins.values()) {
        if (auto pc = plugin->loadPointCloud(fileName)) {
            return pc;
        }
    }

    qWarning(loggingCategory) << "Failed to read PointCloud. No plugin can handle the format of the file:" << fileName;
    return nullptr;
}

} // namespace Z3D
