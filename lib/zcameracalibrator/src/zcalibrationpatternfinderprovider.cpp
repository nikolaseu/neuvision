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

#include "zcalibrationpatternfinderprovider.h"

#include "zcalibrationpatternfinderplugininterface.h"
#include "zcoreplugin.h"
#include "zpluginloader.h"

#include <QDebug>

namespace Z3D
{

QMap< QString, ZCalibrationPatternFinderPluginInterface *> ZCalibrationPatternFinderProvider::m_plugins;

void ZCalibrationPatternFinderProvider::loadPlugins()
{
    const auto list = ZPluginLoader::plugins("calibrationpatternfinder");

    for (auto pluginLoader : list) {
        auto *plugin = pluginLoader->instance<ZCalibrationPatternFinderPluginInterface>();
        if (plugin) {
            qDebug() << "pattern finder plugin loaded. type:" << pluginLoader->id();
            m_plugins.insert(pluginLoader->id(), plugin);
        } else {
            qWarning() << "invalid pattern finder plugin:" << plugin;
        }
    }
}

void ZCalibrationPatternFinderProvider::unloadPlugins()
{
    for(auto *plugin : m_plugins.values()) {
        delete plugin;
    }

    m_plugins.clear();
}

QList<ZCalibrationPatternFinderPtr> ZCalibrationPatternFinderProvider::getAll()
{
    QList<ZCalibrationPatternFinderPtr> finderList;

    /// load different calibration pattern finder types
    for (auto *plugin : m_plugins) {
        finderList << plugin->getPatternFinders();
    }

    return finderList;
}

QWidget *ZCalibrationPatternFinderProvider::getConfigWidget(ZCalibrationPatternFinder *patternFinder)
{
    for (auto *plugin : m_plugins) {
        if (auto *widget = plugin->getConfigWidget(patternFinder)) {
            return widget;
        }
    }

    return nullptr;
}

ZCalibrationPatternFinderProvider::ZCalibrationPatternFinderProvider()
{

}

} // namespace Z3D
