/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "ZPointCloud/zpointcloud_fwd.h"
#include "ZPointCloud/zpointcloud_global.h"

#include <QMap>

//! TODO: This is ugly, find a better way to do this
#define Z3D_ZPOINTCLOUD_INIT() \
    Z3D::ZPointCloudProvider::registerMetaTypes(); \
    Z3D::ZPointCloudProvider::loadPlugins()

//! TODO: This is ugly, find a better way to do this
#define Z3D_ZPOINTCLOUD_INIT_QMLENGINE(engine) \
    engine.addImportPath("qrc:/")

namespace Z3D
{

class ZPointCloudPluginInterface;

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloudProvider
{

public:
    static void registerMetaTypes();

    static void loadPlugins();
    static void unloadPlugins();

    static ZPointCloudUniquePtr loadPointCloud(const QString &fileName);

private:
    explicit ZPointCloudProvider() {}

    static QMap< QString, ZPointCloudPluginInterface *> m_plugins;
};

} // namespace Z3D
