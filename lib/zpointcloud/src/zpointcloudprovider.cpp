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
#include "ZPointCloud/zpointcloud_qml.h"
#include "ZPointCloud/zpointcloudgeometry.h"
#include "ZPointCloud/zpointcloudlistmodel.h"
#include "ZPointCloud/zpointcloudplugininterface.h"
#include "ZPointCloud/zpointcloudreader.h"

#include <ZCore/zcoreplugin.h>
#include <ZCore/zlogging.h>
#include <ZCore/zpluginloader.h>

#include <QDir>
#include <QQmlEngine>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zpointcloud", QtInfoMsg)

namespace Z3D
{

QMap< QString, ZPointCloudPluginInterface *> ZPointCloudProvider::m_plugins;

void ZPointCloudProvider::registerMetaTypes()
{
    qRegisterMetaType<Z3D::ZPointCloudListModel *>("Z3D::ZPointCloudListModel*");
    qRegisterMetaType<Z3D::ZPointCloud *>("Z3D::ZPointCloud*");

    qmlRegisterType<Z3D::ZPointCloudReader>("Z3D.ZPointCloud", 1, 0, "PointCloudReader");
    qmlRegisterType<Z3D::ZPointCloudGeometry>("Z3D.ZPointCloud", 1, 0, "PointCloudGeometry");

    qmlRegisterSingletonType<Z3D::ZPointCloudInternal::ZPointCloud_QML>("Z3D.ZPointCloud", 1, 0, "Utils",
        [](QQmlEngine *engine, QJSEngine *scriptEngine) -> QObject * {
            Q_UNUSED(engine)
            Q_UNUSED(scriptEngine)
            return new Z3D::ZPointCloudInternal::ZPointCloud_QML();
        });
}

void ZPointCloudProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("pointcloud");

    for (auto pluginLoader : list) {
        auto *plugin = pluginLoader->instance<ZPointCloudPluginInterface>();
        if (plugin) {
            zDebug() << "pointcloud plugin loaded. type:" << pluginLoader->id();
            m_plugins.insert(pluginLoader->id(), plugin);
        } else {
            zWarning() << "invalid pointcloud plugin:" << plugin;
        }
    }
}

void ZPointCloudProvider::unloadPlugins()
{
    const auto plugins = m_plugins.values();
    for (const auto *plugin : plugins) {
        delete plugin;
    }

    m_plugins.clear();
}

ZPointCloudUniquePtr ZPointCloudProvider::loadPointCloud(const QString &fileName)
{
    zDebug() << "Trying to read PointCloud from" << fileName;

    const auto plugins = m_plugins.values();
    for (const auto *plugin : plugins) {
        if (auto pc = plugin->loadPointCloud(fileName)) {
            return pc;
        }
    }

    zWarning() << "Failed to read PointCloud. No plugin can handle the format of the file:" << fileName;
    return nullptr;
}

} // namespace Z3D
