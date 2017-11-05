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

#include "zcoreplugin.h"

#include <QPluginLoader>

namespace Z3D
{

ZCorePlugin::ZCorePlugin(QString fileName)
    : m_loader(new QPluginLoader(fileName))
{

}

ZCorePlugin::~ZCorePlugin()
{
    delete m_loader;
}

QString ZCorePlugin::id() const
{
    QJsonObject data = metaData();
    if (!data.contains("id")) {
        return instance<QObject>()->metaObject()->className();
    }

    return data["id"].toString();
}

QString ZCorePlugin::version() const
{
    QJsonObject data = metaData();
    if (!data.contains("version")) {
        return QLatin1String(Z3D_VERSION_STR);
    }

    return data["version"].toString();
}

QJsonObject ZCorePlugin::metaData() const
{
    return m_loader->metaData()["MetaData"].toObject();
}

bool ZCorePlugin::load()
{
    if (!m_loader->load()) {
        return false;
    }

    m_pluginInstance = m_loader->instance();
    return true;
}

QString ZCorePlugin::errorString()
{
    return m_loader->errorString();
}

} // namespace Z3D
