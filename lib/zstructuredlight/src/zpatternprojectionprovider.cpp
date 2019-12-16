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

#include "ZStructuredLight/zpatternprojectionprovider.h"

#include "ZStructuredLight/zpatternprojectionplugin.h"

#include "ZCore/zcoreplugin.h"
#include "ZCore/zpluginloader.h"

#include <QDebug>
#include <QSettings>

namespace Z3D
{

std::map< QString, ZPatternProjectionPlugin *> ZPatternProjectionProvider::m_plugins;

void ZPatternProjectionProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("structuredlightpatterns");

    for (auto pluginLoader : list) {
        auto *plugin = pluginLoader->instance<ZPatternProjectionPlugin>();
        if (plugin) {
            qDebug() << "pattern projection plugin loaded. type:" << pluginLoader->id();
            m_plugins[pluginLoader->id()] = plugin;
        } else {
            qWarning() << "invalid pattern projection plugin:" << plugin;
        }
    }
}

void ZPatternProjectionProvider::unloadPlugins()
{

}

ZPatternProjectionPtr ZPatternProjectionProvider::get(QSettings *settings)
{
    ZPatternProjectionPtr patternProjection;

    settings->beginGroup("PatternProjection");
    {
        const QString pluginId = settings->value("Type").toString();
        if (m_plugins.find(pluginId) != m_plugins.end()) {
            const auto plugin = m_plugins[pluginId];
            patternProjection = plugin->get(settings);
        } else {
            qWarning() << "pattern projection type not found:" << pluginId;
        }
    }
    settings->endGroup();

    return patternProjection;
}

ZPatternProjectionProvider::ZPatternProjectionProvider()
{
}

} // namespace Z3D
