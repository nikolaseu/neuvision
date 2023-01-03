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

#include "zqtcameraplugin.h"

#include "zqtcamera.h"

#include "ZCameraAcquisition/zcamerainfo.h"
#include "ZCore/zlogging.h"

#include <QtMultimedia/QCameraDevice>
#include <QtMultimedia/QMediaDevices>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameraacquisition.zqtcamera", QtInfoMsg)

namespace Z3D
{

namespace
{
Q_GLOBAL_STATIC(QMediaDevices, s_devices)
}

ZQtCameraPlugin::ZQtCameraPlugin()
{
    updateCameras(); /// updateCameras() triggers a QMediaDevices::videoInputsChanged() so it must be called before connecting to the notification signal, otherwise it might hung (in Qt 6.5 beta 1)

    QObject::connect(s_devices, &QMediaDevices::videoInputsChanged,
                     this, &ZQtCameraPlugin::updateCameras);
}

QString ZQtCameraPlugin::displayName() const
{
    return tr("QtMultimedia camera");
}

QList<ZCameraInfo *> ZQtCameraPlugin::getConnectedCameras()
{
    return m_cameraList;
}

ZCameraPtr ZQtCameraPlugin::getCamera(QVariantMap options)
{
    const QByteArray cameraId = options["id"].toByteArray();

    if (cameraId.isNull() || cameraId.isEmpty()) {
        return nullptr;
    }

    const QList<QCameraDevice> cameras = s_devices->videoInputs();
    for (const QCameraDevice &cameraDevice : cameras) {
        if (cameraDevice.id() == cameraId) {
            auto *camera = new QCamera(cameraDevice);
            if (camera->isAvailable()) {
                return ZCameraPtr(new ZQtCamera(camera));
            }
            delete camera;
            break;
        }
    }

    return nullptr;
}

void ZQtCameraPlugin::updateCameras()
{
    for (auto &cameraInfo : m_cameraList) {
        cameraInfo->deleteLater();
    }
    m_cameraList.clear();

    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    zDebug() << "found" << cameras.size() << "cameras";

    for (const QCameraDevice &cameraDevice : cameras) {
        QVariantMap extraData;
        extraData["description"] = cameraDevice.description();
        extraData["id"] = cameraDevice.id();
        m_cameraList << new ZCameraInfo(this, cameraDevice.description(), extraData);
    }
}

} // namespace Z3D
