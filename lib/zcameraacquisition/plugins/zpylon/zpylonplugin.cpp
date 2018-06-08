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

#include "zpylonplugin.h"

#include "zpyloncamera.h"
#include "zcamerainfo.h"

#include <pylon/PylonIncludes.h>

#include <QDebug>
#include <QStringList>

namespace Z3D
{

ZPylonPlugin::ZPylonPlugin()
{
    Pylon::PylonInitialize();
}

ZPylonPlugin::~ZPylonPlugin()
{
    Pylon::PylonTerminate();
}

QString ZPylonPlugin::displayName() const
{
    return QString("Basler Pylon SDK");
}

QList<ZCameraInfo *> ZPylonPlugin::getConnectedCameras()
{
    QList<ZCameraInfo *> camerasList;

    Pylon::DeviceInfoList_t devices;
    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

    if (tlFactory.EnumerateDevices(devices) == 0) {
        qWarning() << "failed to enumerate devices";
        return camerasList;
    }

    for (const auto &device : devices) {
        QVariantMap extraData;
        extraData["SerialNumber"] = device.GetSerialNumber().c_str();
        extraData["FullName"] = device.GetFullName().c_str();
        camerasList << new ZCameraInfo(this, device.GetSerialNumber().c_str(), extraData);
    }

    return camerasList;
}

ZCameraPtr ZPylonPlugin::getCamera(QVariantMap options)
{
    QString serialNumber = options.value("SerialNumber").toString();
    QString fullName;

    auto cameras = getConnectedCameras();
    for (const auto *cameraInfo : cameras) {
        const auto extraData = cameraInfo->extraData();
        QString infoSerialNumber = extraData.value("SerialNumber").toString();
        if (serialNumber.compare(infoSerialNumber) == 0) {
            fullName = extraData.value("FullName").toString();
            break;
        }
    }

    if (fullName.isNull()) {
        return nullptr;
    }

    auto *device = Pylon::CTlFactory::GetInstance()
            .CreateDevice(Pylon::CDeviceInfo().SetFullName(qPrintable(fullName)));

    if (!device) {
        return nullptr;
    }

    ZCameraPtr camera(new PylonCamera(device));

    if (options.contains("ConfigFile")) {
        QString configFileName = options.value("ConfigFile").toString();
        camera->loadConfiguration(configFileName);
    }

    return camera;
}

} // namespace Z3D
