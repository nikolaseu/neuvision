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

#include <QCameraInfo>

namespace Z3D {

QString ZQtCameraPlugin::displayName() const
{
    return tr("QtMultimedia camera");
}

QList<ZCameraInfo *> ZQtCameraPlugin::getConnectedCameras()
{
    QList<ZCameraInfo *> cameraList;

    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    qDebug() << "found" << cameras.size() << "cameras.";
    for (const QCameraInfo &qtcameraInfo : cameras) {
        QVariantMap extraData;
        extraData["description"] = qtcameraInfo.description();
        extraData["deviceName"] = qtcameraInfo.deviceName();
        cameraList << new ZCameraInfo(this, qtcameraInfo.description(), extraData);
    }

    return cameraList;
}

ZCameraPtr ZQtCameraPlugin::getCamera(QVariantMap options)
{
    QByteArray deviceName = options["deviceName"].toByteArray();

    if (deviceName.isNull() || deviceName.isEmpty()) {
        return nullptr;
    }

    QCamera *qcamera = new QCamera(deviceName);
    if (qcamera->isAvailable()) {
        return ZCameraPtr(new ZQtCamera(qcamera));
    }

    delete qcamera;
    return nullptr;
}

} // namespace Z3D
