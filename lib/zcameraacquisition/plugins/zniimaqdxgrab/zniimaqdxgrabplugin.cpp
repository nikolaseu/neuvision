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

#include "zniimaqdxgrabplugin.h"

#include "zniimaqdxgrabcamera.h"

#include "NIIMAQdx.h"
//#include "nivision.h"

#include <QDebug>
#include <QStringList>

namespace Z3D
{

QString ZNIIMAQdxGrabPlugin::displayName()
{
    return QString("ZNIIMAQdxGrab");
}

QString ZNIIMAQdxGrabPlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QStringList ZNIIMAQdxGrabPlugin::getConnectedCameras()
{
    IMAQdxError status = IMAQdxErrorSuccess;
    uInt32 numCameras;
    bool connectedOnly = true;

    QStringList camerasList;

    status = IMAQdxEnumerateCameras(NULL, &numCameras, connectedOnly);
    if (status) {
        qWarning() << "failed to obtain connected cameras information";
        return camerasList;
    }

    if (numCameras > 0) {
        qDebug() << numCameras << "connected cameras found";

        IMAQdxCameraInformation *camerasInfo = new IMAQdxCameraInformation[numCameras];
        status = IMAQdxEnumerateCameras(camerasInfo, &numCameras, connectedOnly);

        if (status) {
            qWarning() << "failed to obtain cameras information";
            return camerasList;
        }

        for (unsigned int iCam=0; iCam<numCameras; ++iCam) {
            qDebug() << "__________________________________________________________";
            qDebug() << "CameraAttributeURL" << camerasInfo[iCam].CameraAttributeURL;
            qDebug() << "CameraFileName    " << camerasInfo[iCam].CameraFileName;
            qDebug() << "InterfaceName     " << camerasInfo[iCam].InterfaceName;
            qDebug() << "ModelName         " << camerasInfo[iCam].ModelName;
            qDebug() << "SerialNumberHi    " << camerasInfo[iCam].SerialNumberHi;
            qDebug() << "SerialNumberLo    " << camerasInfo[iCam].SerialNumberLo;
            qDebug() << "VendorName        " << camerasInfo[iCam].VendorName;
            qDebug() << "Version           " << camerasInfo[iCam].Version;

//            emit cameraConnected(camerasInfo[iCam].InterfaceName);

            camerasList << camerasInfo[iCam].InterfaceName;
        }
    }

    return camerasList;
}

ZCameraPtr ZNIIMAQdxGrabPlugin::getCamera(QVariantMap options)
{
    QString cameraName = options.value("Name").toString();
    return ZNIIMAQdxGrabCamera::getCameraByName(cameraName);
}

} // namespace Z3D
