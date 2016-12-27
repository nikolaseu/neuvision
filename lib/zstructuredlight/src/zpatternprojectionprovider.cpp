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

#include "zpatternprojectionprovider.h"

#include "zpluginloader.h"
#include "zpatternprojectionplugin.h"

#include <QDebug>

namespace Z3D
{

QList<ZPatternProjectionPlugin*> ZPatternProjectionProvider::m_list;

void ZPatternProjectionProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("structuredlightpatterns");

    for (auto pluginInstance : list) {
        auto *plugin = qobject_cast<ZPatternProjectionPlugin *>(pluginInstance);
        if (plugin) {
            qDebug() << "pattern projection plugin loaded. type:" << plugin->id()
                     << "version:" << plugin->version();
            m_list << plugin;
        } else {
            qWarning() << "invalid pattern projection plugin:" << plugin;
        }
    }
}

void ZPatternProjectionProvider::unloadPlugins()
{

}

QList<ZPatternProjection *> ZPatternProjectionProvider::getAll()
{
    QList<ZPatternProjection *> list;
    for (auto plugin : m_list) {
        list << plugin->getAll();
    }
    return list;
}

ZPatternProjectionProvider::ZPatternProjectionProvider()
{
}

} // namespace Z3D
