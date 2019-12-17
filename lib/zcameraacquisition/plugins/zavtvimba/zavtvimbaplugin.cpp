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

#include "zavtvimbaplugin.h"

#include "zavtvimbacamera.h"

#include "ZCameraAcquisition/zcamerainfo.h"

#include <QDebug>
#include <VimbaCPP/Include/VimbaSystem.h>

namespace Z3D
{

ZAVTVimbaPlugin::ZAVTVimbaPlugin()
{
    /// initialize vimba system
    VmbErrorType err = AVT::VmbAPI::VimbaSystem::GetInstance().Startup();
    if ( VmbErrorSuccess != err )
        qWarning() << "AVT::VmbAPI::VimbaSystem::GetInstance().Startup() failed. Error:" << err;
}

ZAVTVimbaPlugin::~ZAVTVimbaPlugin()
{
    /// shutdown vimba system
    AVT::VmbAPI::VimbaSystem::GetInstance().Shutdown();
}

QString ZAVTVimbaPlugin::displayName() const
{
    return QString("AVT Vimba SDK");
}

QList<ZCameraInfo *> ZAVTVimbaPlugin::getConnectedCameras()
{
    QList<ZCameraInfo*> camerasList;

    AVT::VmbAPI::CameraPtrVector cameras;
    /// Get all known cameras
    if ( VmbErrorSuccess == AVT::VmbAPI::VimbaSystem::GetInstance().GetCameras( cameras )) {
        /// reserve space
        camerasList.reserve(cameras.size());

        /// And query the camera details
        for (AVT::VmbAPI::CameraPtrVector::const_iterator iter = cameras.begin(); iter != cameras.end(); ++iter) {
            AVTVimbaCamera camera(*iter);

            QVariantMap extraData;
            extraData["DeviceID"] = camera.deviceID();

            //! FIXME use a friendlyier name for the camera instead of the DeviceID
            camerasList.push_back( new ZCameraInfo(this, camera.deviceID(), extraData) );
        }
    } else {
        qWarning() << "Unable to obtain cameras. AVT::VmbAPI::VimbaSystem::GetInstance().GetCameras() failed.";
    }

    return camerasList;
}

ZCameraPtr ZAVTVimbaPlugin::getCamera(QVariantMap options)
{
    QString cameraDeviceID = options.value("DeviceID").toString();

    AVTVimbaCamera *avtCamera = nullptr;

    AVT::VmbAPI::CameraPtrVector cameras;
    /// Get all known cameras
    if ( VmbErrorSuccess == AVT::VmbAPI::VimbaSystem::GetInstance().GetCameras( cameras )) {
        /// query the camera details
        for (AVT::VmbAPI::CameraPtrVector::const_iterator iter = cameras.begin(); iter != cameras.end(); ++iter) {
            avtCamera = new AVTVimbaCamera(*iter);

            qDebug() << "camera:" << avtCamera->deviceID();

            if (avtCamera->deviceID() == cameraDeviceID)
                break;

            delete avtCamera;
            avtCamera = nullptr;
        }
    } else {
        qWarning() << "Unable to obtain cameras. AVT::VmbAPI::VimbaSystem::GetInstance().GetCameras() failed.";
    }

    if (avtCamera) {
        return ZCameraPtr(avtCamera);
    }

    return nullptr;
}

} // namespace Z3D
