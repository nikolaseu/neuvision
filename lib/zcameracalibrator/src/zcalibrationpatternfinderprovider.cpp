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
#include "zpluginloader.h"

#include <QDebug>

namespace Z3D
{

QMap< QString, ZCalibrationPatternFinderPluginInterface *> ZCalibrationPatternFinderProvider::m_plugins;

void ZCalibrationPatternFinderProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("calibrationpatternfinder");

    for (auto pluginInstance : list) {
        auto *plugin = qobject_cast<ZCalibrationPatternFinderPluginInterface *>(pluginInstance);
        if (plugin) {
            qDebug() << "pattern finder plugin loaded. type:" << plugin->id()
                     << "version:" << plugin->version();
            m_plugins.insert(plugin->id(), plugin);
        } else {
            qWarning() << "invalid pattern finder plugin:" << plugin;
        }
    }
}

void ZCalibrationPatternFinderProvider::unloadPlugins()
{
    foreach(ZCalibrationPatternFinderPluginInterface *plugin, m_plugins.values())
        delete plugin;

    m_plugins.clear();
}

QList<ZCalibrationPatternFinder::Ptr> ZCalibrationPatternFinderProvider::getAll()
{
    QList<ZCalibrationPatternFinder::Ptr> finderList;

    /// load different calibration pattern finder types
    foreach (ZCalibrationPatternFinderPluginInterface *plugin, m_plugins)
        finderList << plugin->getPatternFinders();

    return finderList;
}

ZCalibrationPatternFinderProvider::ZCalibrationPatternFinderProvider()
{

}

} // namespace Z3D
