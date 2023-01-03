/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include "ZCore/zcore_global.h"

#include <QtCore/QObject>

class QPluginLoader;

namespace Z3D
{

class Z3D_CORE_SHARED_EXPORT ZCorePlugin
{

public:
    explicit ZCorePlugin(const QString &fileName);
    virtual ~ZCorePlugin();

    /// plugin information
    QString id() const;
    QString version() const;
    QJsonObject metaData() const;

    /// Loads the plugin and returns true if the plugin was loaded successfully;
    /// otherwise returns false
    bool load();

    /// Destroys instance and unloads the plugin
    void unload();

    /// Returns a text string with the description of the last error that occurred
    QString errorString();

    /// plugin instance
    template <class ZPlugin>
    ZPlugin *instance()
    {
        if (!m_pluginInstance && !load()) {
            return nullptr;
        }

        return qobject_cast<ZPlugin *>(m_pluginInstance);
    }

private:
    QPluginLoader *const m_loader;
    QObject *m_pluginInstance = nullptr;
};

} // namespace Z3D
