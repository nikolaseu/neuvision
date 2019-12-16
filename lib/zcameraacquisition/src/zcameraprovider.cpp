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

#include "ZCameraAcquisition/zcameraprovider.h"

#include "ZCameraAcquisition/zcamerainterface.h"
#include "ZCameraAcquisition/zcameralistmodel.h"
#include "ZCameraAcquisition/zcameraplugininterface.h"

#include "ZCore/zcoreplugin.h"
#include "ZCore/zpluginloader.h"

#include <QDebug>
#include <QDir>

namespace Z3D
{

QMap< QString, ZCameraPluginInterface *> ZCameraProvider::m_plugins;
Z3D::ZCameraListModel ZCameraProvider::m_model;

void ZCameraProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("cameraacquisition");

    for (auto pluginLoader : list) {
        auto *plugin = pluginLoader->instance<ZCameraPluginInterface>();
        if (plugin) {
            qDebug() << "camera plugin loaded. type:" << pluginLoader->id();
            m_plugins.insert(pluginLoader->id(), plugin);
        } else {
            qWarning() << "invalid camera plugin:" << plugin;
        }
    }
}

void ZCameraProvider::unloadPlugins()
{
    for (const auto *plugin : m_plugins.values()) {
        delete plugin;
    }

    m_plugins.clear();
}

ZCameraPtr ZCameraProvider::getCamera(QString pluginName, QVariantMap options)
{
    ZCameraPtr camera;

    if (m_plugins.contains(pluginName)) {
        camera = m_plugins[pluginName]->getCamera(options);

        if (camera) {
            /// load configuration file
            if (options.contains("ConfigFile")) {
                QString configFileName = options.value("ConfigFile").toString();
                camera->loadConfiguration(configFileName);
            }

            /// initial preset to load ("BaseAcquisitionMode" is always loaded)
            if (options.contains("Preset")) {
                QString presetName = options.value("Preset").toString();
                camera->loadPresetConfiguration(presetName);
            }

            /// custom buffer size
            if (options.contains("BufferSize")) {
                int bufferSize = options.value("BufferSize").toInt();
                if (camera->setBufferSize(bufferSize)) {
                    qDebug() << "camera" << camera->uuid() << "using a buffer size of" << bufferSize;
                }
            }

            m_model.add(camera);
        }
    } else {
        qWarning() << "camera plugin not found:" << pluginName
                   << "available plugins:" << m_plugins.keys();
    }

    return camera;
}

ZCameraPtr ZCameraProvider::getCamera(QSettings *settings)
{
    QVariantMap options;
    for (const auto &key : settings->allKeys()) {
        options[key] = settings->value(key);
    }

    QString deviceType = options["Type"].toString();

    return getCamera(deviceType, options);
}

ZCameraList ZCameraProvider::loadCameras(QString folder)
{
    QDir configDir = QDir(folder);

    /// if no folder is indicated, use current running folder and standard search path
    if (folder.isEmpty()) {
        configDir = QDir::current();

    #if defined(Q_OS_WIN)
//        if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
//            pluginsDir.cdUp();
    #elif defined(Q_OS_MAC)
        if (configDir.dirName() == "MacOS") {
            configDir.cdUp();
            configDir.cdUp();
            configDir.cdUp();
        }
    #endif

        if (!configDir.cd("cameras")) {
            qWarning() << "cameras configuration folder 'cameras' not found in" << configDir.absolutePath();
            return ZCameraList();
        }
    }

    qDebug() << "loading cameras from" << configDir.absolutePath();

    ZCameraList cameraList;

    QStringList filters;
    filters << "*.ini" << "*.json"; //! TODO: agregar para leer configuracion en JSON
    configDir.setNameFilters(filters);

    for (const auto &fileName : configDir.entryList(QDir::Files)) {
        qDebug() << "found" << fileName;
        QSettings settings(configDir.absoluteFilePath(fileName), QSettings::IniFormat);
        settings.beginGroup("Camera");
        cameraList.push_back(getCamera(&settings));
        settings.endGroup();
    }

    return cameraList;
}

ZCameraListModel *ZCameraProvider::model()
{
    return &m_model;
}

QList<ZCameraPluginInterface *> ZCameraProvider::availablePlugins()
{
    QList<ZCameraPluginInterface *> pluginList;

    for (auto *plugin : m_plugins.values()) {
        pluginList << plugin;
    }

    return pluginList;
}


} // namespace Z3D
