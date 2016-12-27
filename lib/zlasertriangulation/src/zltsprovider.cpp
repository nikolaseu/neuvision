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

#include "zltsprovider.h"

#include "zltscamera3d.h"
#include "zltsplanecalibration.h"

#include <Z3DCalibratedCamera>
#include <Z3DCameraAcquisition>
#include <Z3DCameraCalibration>

#include <QDebug>
#include <QDir>
//#include <QPluginLoader>
#include <QSettings>

namespace Z3D
{

QList<Z3D::LTSCamera3D::Ptr> LTSProvider::loadCameras(QString folder)
{
    /// if no folder is indicated, use current running folder + "/cameras"
    if (folder.isEmpty()) {
        folder = QDir::current().absoluteFilePath("cameras");
    }

    QDir configDir(folder);

    qDebug() << "loading cameras from" << configDir.absolutePath();

    QList<Z3D::LTSCamera3D::Ptr> cameraList;

    QStringList filters;
    filters << "*.ini" << "*.json"; //! TODO: agregar para leer configuracion en JSON
    configDir.setNameFilters(filters);

    foreach (QString fileName, configDir.entryList(QDir::Files)) {
        qDebug() << "found" << fileName;
        QSettings settings(configDir.absoluteFilePath(fileName), QSettings::IniFormat);

        /// this is only valid for "ProfileCamera" type
        QString ltsCameraType = settings.value("LaserTriangulationSystem/Type").toString();
        if (ltsCameraType == "ProfileCamera") {
        } else {
            qWarning() << "unknown LTS camera type:" << ltsCameraType;
            continue;
        }

        /// 2d camera
        Z3D::ZCameraInterface::Ptr camera;
        Z3D::ZCameraCalibration::Ptr cameraCalibration;
        Z3D::ZCalibratedCamera::Ptr calibratedCamera;

        /// lts system
        LTSCamera3DCalibrationInterface::Ptr ltsCalibration;
        Z3D::LTSCamera3D::Ptr ltsCamera;

        /// camera device
        settings.beginGroup("Camera");
        {
            camera = ZCameraProvider::getCamera(&settings);
            if (!camera) {
                /// we can't do anything without a camera
                qWarning() << "unable to load camera";
                continue;
            }
        }
        settings.endGroup();

        /// camera calibration
        settings.beginGroup("CameraCalibration");
        {
            cameraCalibration = ZCameraCalibrationProvider::getCalibration(&settings);
            if (!cameraCalibration) {
                qWarning() << "unable to load camera calibration";
            }
        }
        settings.endGroup();

        /// create calibrated camera
        calibratedCamera = Z3D::ZCalibratedCamera::Ptr(new Z3D::ZCalibratedCamera(camera, cameraCalibration));

        /// lts calibration
        settings.beginGroup("LaserTriangulationSystem");
        {
            settings.beginGroup("Calibration");
            {
                QString ltsCalType = settings.value("Type").toString();
                if (ltsCalType == "Plane") {
                    Z3D::LTSPlaneCalibration *ltsPlaneCalibration = new Z3D::LTSPlaneCalibration(cameraCalibration);
                    QString ltsCalFile = settings.value("ConfigFile").toString();
                    if (!ltsCalFile.isEmpty())
                        ltsPlaneCalibration->loadCalibration(ltsCalFile);
                    ltsCalibration = LTSCamera3DCalibrationInterface::Ptr(ltsPlaneCalibration);
                } else {
                    qWarning() << "unknown LTS calibration type:" << ltsCalType;
                }
            }
            settings.endGroup();

            /// lts camera
            ltsCamera = Z3D::LTSCamera3D::Ptr(new Z3D::LTSCamera3D(calibratedCamera, ltsCalibration));
            /// start in profile mode
            ltsCamera->setAcquisitionMode(Z3D::LTSCamera3D::ProfileAcquisitionMode);
        }
        settings.endGroup();

        /// add to list
        cameraList << ltsCamera;
    }

    return cameraList;
}

} // namespace Z3D
