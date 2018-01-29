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
#include "zcoreplugin.h"
#include "zpinhole/zpinholecameracalibrationplugin.h"
#include "zpluginloader.h"

#include <QDebug>
#include <QSettings>

namespace Z3D
{

QMap< QString, ZCameraCalibrationPluginInterface *> ZCameraCalibrationProvider::m_plugins;

void ZCameraCalibrationProvider::loadPlugins()
{
    /// add pinhole camera calibration plugin
    ZCameraCalibrationPluginInterface *pinholePlugin = new ZPinholeCameraCalibrationPlugin;
    m_plugins.insert(QLatin1String("Pinhole"), pinholePlugin);

    const auto list = ZPluginLoader::plugins(QLatin1String("cameracalibration"));

    for (auto pluginLoader : list) {
        auto *plugin = pluginLoader->instance<ZCameraCalibrationPluginInterface>();
        if (plugin) {
            qDebug() << "camera calibration plugin loaded. type:" << pluginLoader->id();
            m_plugins.insert(pluginLoader->id(), plugin);
        } else {
            qWarning() << "invalid camera calibration plugin:" << plugin;
        }
    }
}

void ZCameraCalibrationProvider::unloadPlugins()
{
    for (auto *plugin : m_plugins.values()) {
        delete plugin;
    }

    m_plugins.clear();
}

ZCameraCalibrationPtr ZCameraCalibrationProvider::getCalibration(QString pluginName, QVariantMap options)
{
    if (!m_plugins.contains(pluginName)) {
        qWarning() << "camera calibration plugin not found:" << pluginName;
        return nullptr;
    }

    return m_plugins[pluginName]->getCalibration(options);
}

ZCameraCalibrationPtr ZCameraCalibrationProvider::getCalibration(QSettings *settings)
{
    QVariantMap options;
    for (const auto &key : settings->allKeys()) {
        options[key] = settings->value(key);
    }

    QString pluginName = options["Type"].toString();

    return getCalibration(pluginName, options);
}

QList<ZCameraCalibrator *> ZCameraCalibrationProvider::getCameraCalibrators()
{
    QList<ZCameraCalibrator *> list;

    for (auto *plugin : m_plugins.values()) {
        list << plugin->getCameraCalibrators();
    }

    return list;
}

QWidget *ZCameraCalibrationProvider::getConfigWidget(ZCameraCalibrator *cameraCalibrator)
{
    for (auto *plugin : m_plugins.values()) {
        if (auto *configWidget = plugin->getConfigWidget(cameraCalibrator)) {
            return configWidget;
        }
    }

    return nullptr;
}

ZMultiCameraCalibrationPtr ZCameraCalibrationProvider::getMultiCameraCalibration(QString pluginName, QVariantMap options)
{
    if (!m_plugins.contains(pluginName)) {
        qWarning() << "camera calibration plugin not found:" << pluginName;
        return nullptr;
    }

    return m_plugins[pluginName]->getMultiCameraCalibration(options);
}

ZMultiCameraCalibrationPtr ZCameraCalibrationProvider::getMultiCameraCalibration(QSettings *settings)
{
    QVariantMap options;
    for (const auto &key : settings->allKeys()) {
        options[key] = settings->value(key);
    }

    QString pluginName = options["Type"].toString();

    return getMultiCameraCalibration(pluginName, options);
}

bool ZCameraCalibrationProvider::saveCameraCalibration(const QString &fileName, ZMultiCameraCalibrationPtr calibration)
{
    //! FIXME not really the best way
    for (auto *plugin : m_plugins.values()) {
        if (plugin->saveCalibration(fileName, calibration)) {
            return true;
        }
    }

    return false;
}

QList<ZMultiCameraCalibrator *> ZCameraCalibrationProvider::getMultiCameraCalibrators()
{
    QList<ZMultiCameraCalibrator *> list;

    for (auto *plugin : m_plugins.values()) {
        list << plugin->getMultiCameraCalibrators();
    }

    return list;
}

QWidget *ZCameraCalibrationProvider::getConfigWidget(ZMultiCameraCalibrator *multiCameraCalibrator)
{
    for (auto *plugin : m_plugins.values()) {
        if (auto *configWidget = plugin->getConfigWidget(multiCameraCalibrator)) {
            return configWidget;
        }
    }

    return nullptr;
}

} // namespace Z3D
