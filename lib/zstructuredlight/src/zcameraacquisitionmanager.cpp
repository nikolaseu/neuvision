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

#include "ZStructuredLight/zcameraacquisitionmanager.h"

#include "ZCameraAcquisition/zcameraimage.h"
#include "ZCameraAcquisition/zcamerainterface.h"

#include <QDir>

namespace Z3D {

ZCameraAcquisitionManager::ZCameraAcquisitionManager(ZCameraList cameras, QObject *parent)
    : QObject(parent)
    , m_cameras(cameras)
{

}

void ZCameraAcquisitionManager::prepareAcquisition(QString acquisitionId)
{
    m_acquisitionId = acquisitionId;

    /// clear previous images
    m_images.clear();

    ///
    if (m_debugMode) {
        for (auto cam : m_cameras) {
            QString cameraFolder = QString("%1/%2")
                    .arg(m_acquisitionId)
                    .arg(cam->uuid());
            if (!QDir::current().mkpath(cameraFolder)) {
                qWarning() << "Unable to create folder:" << cameraFolder;
                return;
            }
        }
    }

    /// start acquisition
    for (auto cam : m_cameras) {
        cam->startAcquisition();
    }

    emit acquisitionReady(m_acquisitionId);
}

void ZCameraAcquisitionManager::acquireSingle(QString id)
{
    for (auto cam : m_cameras) {
        /// if the camera is simulated, set which image should use now
        if (cam->uuid().startsWith("ZSIMULATED")) {
            qDebug() << "camera is simulated. using image" << id;
            cam->setAttribute("CurrentFile", id);
        }

        /// schedule an image request, the camera should manage the snapshot asynchronously
        cam->requestSnapshot();
    }

    m_images.push_back(std::vector<Z3D::ZCameraImagePtr>());
    auto &images = m_images.back();
    images.reserve(m_cameras.size());

    for (auto cam : m_cameras) {
        /// Retrieve the image, this will block until the snapshot is retrieved
        auto imptr = cam->getSnapshot();

        if (imptr) {
            /// create a deep copy, the camera might return a temporary
            imptr = imptr->clone();

            if (m_debugMode) {
                /// save image
                ZCameraImage::save(imptr, QString("%1/%2/%3")
                            .arg(m_acquisitionId)
                            .arg(cam->uuid())
                            .arg(id));
            }
        } else {
            qCritical() << "error obtaining snapshot for camera" << cam->uuid();
        }

        images.push_back(imptr);
    }

    emit imagesAcquired(images, id);
}

void ZCameraAcquisitionManager::finishAcquisition()
{
    /// stop acquisition
    for (auto cam : m_cameras) {
        cam->stopAcquisition();
    }

    emit acquisitionFinished(m_images, m_acquisitionId);
}

} // namespace Z3D {
