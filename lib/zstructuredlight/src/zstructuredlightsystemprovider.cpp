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

#include "ZStructuredLight/zstructuredlightsystemprovider.h"

#include "ZStructuredLight/zstructuredlightsystemplugin.h"

#include "ZCore/zcoreplugin.h"
#include "ZCore/zlogging.h"
#include "ZCore/zpluginloader.h"

#include <QSettings>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zstructuredlight", QtInfoMsg)

namespace Z3D
{

QMap<QString, ZStructuredLightSystemPlugin *> ZStructuredLightSystemProvider::m_plugins;

void ZStructuredLightSystemProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("structuredlight");

    for (auto pluginLoader : list) {
        auto *plugin = pluginLoader->instance<ZStructuredLightSystemPlugin>();
        if (plugin) {
            zDebug() << "structured light system plugin loaded. type:" << pluginLoader->id();
            m_plugins[pluginLoader->id()] = plugin;
        } else {
            zWarning() << "invalid structured light system plugin:" << plugin;
        }
    }
}

void ZStructuredLightSystemProvider::unloadPlugins()
{

}

ZStructuredLightSystemPtr ZStructuredLightSystemProvider::get(QSettings *settings)
{
    ZStructuredLightSystemPtr structuredLightSystem;

    settings->beginGroup("StructuredLightSystem");
    {
        const QString pluginId = settings->value("Type").toString();
        if (m_plugins.find(pluginId) != m_plugins.end()) {
            const auto plugin = m_plugins[pluginId];
            structuredLightSystem = plugin->get(settings);
        } else {
            QStringList typesList;
            for (const auto &key : m_plugins.keys()) {
                typesList << key;
            }
            zWarning() << "structured light type not found:" << pluginId
                       << "(available types:" << typesList.join(", ") << ")";
            settings->setValue("Type", QString("<FIXME chose one of: %1>").arg(typesList.join(", ")));
        }
    }
    settings->endGroup();

    return structuredLightSystem;
}

ZStructuredLightSystemProvider::ZStructuredLightSystemProvider()
{

}

} // namespace Z3D
