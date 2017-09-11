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

#include "zlucamplugin.h"

#include "zlucamcamera.h"

#include <QDebug>
#include <QStringList>

namespace Z3D
{

QString ZLuCamPlugin::id() const
{
    return QString("ZLuCam");
}

QString ZLuCamPlugin::name() const
{
    return QString("Lumenera LuCam SDK");
}

QString ZLuCamPlugin::version() const
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo *> ZLuCamPlugin::getConnectedCameras()
{
    QList<ZCameraInfo *> camerasList;

    QList<ZCameraInterface::Ptr> cameras =  LuCamCamera::getConnectedCameras();
    foreach (ZCameraInterface::Ptr camera, cameras) {
        QVariantMap extraData;
        extraData["Name"] = camera->uuid();
        camerasList << new ZCameraInfo(this, camera->uuid(), extraData);
    }

    return camerasList;
}

ZCameraInterface::Ptr ZLuCamPlugin::getCamera(QVariantMap options)
{
    QString cameraName = options.value("Name").toString();
    ZCameraInterface::Ptr camera = LuCamCamera::getCameraByName(cameraName);

    if (camera && options.contains("ConfigFile")) {
        QString configFileName = options.value("ConfigFile").toString();
        camera->loadConfiguration(configFileName);
    }

    return camera;
}

} // namespace Z3D
