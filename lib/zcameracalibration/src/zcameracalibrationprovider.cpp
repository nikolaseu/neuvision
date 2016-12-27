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

#include "zcameracalibrationprovider.h"

#include "zcameracalibration.h"
#include "zcameracalibrationplugininterface.h"
#include "zpinhole/zpinholecameracalibrationplugin.h"
#include "zpluginloader.h"

#include <QDebug>

namespace Z3D
{

QMap< QString, ZCameraCalibrationPluginInterface *> ZCameraCalibrationProvider::m_plugins;

void ZCameraCalibrationProvider::loadPlugins(QString folder)
{
    /// add pinhole camera calibration plugin
    ZCameraCalibrationPluginInterface *pinholePlugin = new ZPinholeCameraCalibrationPlugin;
    m_plugins.insert(pinholePlugin->name(), pinholePlugin);

    auto list = ZPluginLoader::plugins("cameracalibration");

    for (auto pluginInstance : list) {
        auto *plugin = qobject_cast<ZCameraCalibrationPluginInterface *>(pluginInstance);
        if (plugin) {
            qDebug() << "camera calibration plugin loaded. type:" << plugin->id()
                     << "version:" << plugin->version();
            m_plugins.insert(plugin->id(), plugin);
        } else {
            qWarning() << "invalid camera calibration plugin:" << plugin;
        }
    }
}

void ZCameraCalibrationProvider::unloadPlugins()
{
    foreach(ZCameraCalibrationPluginInterface *plugin, m_plugins.values())
        delete plugin;

    m_plugins.clear();
}

ZCameraCalibration::Ptr ZCameraCalibrationProvider::getCalibration(QString pluginName, QVariantMap options)
{
    ZCameraCalibration::Ptr calibration;

    if (m_plugins.contains(pluginName)) {
        calibration = m_plugins[pluginName]->getCalibration(options);
    } else {
        qWarning() << "camera calibration plugin not found:" << pluginName;
    }

    return calibration;
}

ZCameraCalibration::Ptr ZCameraCalibrationProvider::getCalibration(QSettings *settings)
{
    QVariantMap options;
    foreach(QString key, settings->allKeys()) {
        options[key] = settings->value(key);
    }

    QString pluginName = options["Type"].toString();

    return getCalibration(pluginName, options);
}

QList<ZCameraCalibrator *> ZCameraCalibrationProvider::getCameraCalibrators()
{
    QList<ZCameraCalibrator *> list;

    foreach(ZCameraCalibrationPluginInterface *plugin, m_plugins.values())
        list << plugin->getCameraCalibrators();

    return list;
}

QList<ZMultiCameraCalibrator *> ZCameraCalibrationProvider::getMultiCameraCalibrators()
{
    QList<ZMultiCameraCalibrator *> list;

    foreach(ZCameraCalibrationPluginInterface *plugin, m_plugins.values())
        list << plugin->getMultiCameraCalibrators();

    return list;
}

} // namespace Z3D
