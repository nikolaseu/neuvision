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

#include "zcalibratedcameraprovider.h"

#include <Z3DCameraInterface>
#include <Z3DCameraProvider>
#include <Z3DCameraCalibrationProvider>

#include <QDebug>
#include <QDir>
#include <QSettings>

namespace Z3D
{

CalibratedCameraProvider::CalibratedCameraProvider()
{

}

QList<Z3D::ZCalibratedCamera::Ptr> CalibratedCameraProvider::loadCameras(QString folder)
{
    auto configDir = QDir(folder);

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
            return QList<Z3D::ZCalibratedCamera::Ptr>();
        }
    }

    qDebug() << "loading cameras from" << configDir.absolutePath();

    QList<Z3D::ZCalibratedCamera::Ptr> cameraList;

    QStringList filters;
    filters << "*.ini" << "*.json"; //! TODO: agregar para leer configuracion en JSON
    configDir.setNameFilters(filters);

    foreach (QString fileName, configDir.entryList(QDir::Files)) {
        qDebug() << "found" << fileName;
        QSettings settings(configDir.absoluteFilePath(fileName), QSettings::IniFormat);

        ZCalibratedCamera::Ptr calibratedCamera = getCalibratedCamera(&settings);

        if (!calibratedCamera) {
            qWarning() << "unable to load calibrated camera";
            continue;
        }

        /// add to list
        cameraList << Z3D::ZCalibratedCamera::Ptr(calibratedCamera);
    }

    return cameraList;
}

ZCalibratedCamera::Ptr CalibratedCameraProvider::getCalibratedCamera(QSettings *settings)
{
    Z3D::ZCameraInterface::Ptr camera;
    Z3D::ZCameraCalibration::Ptr cameraCalibration;

    /// camera
    settings->beginGroup("Camera");
    {
        camera = ZCameraProvider::getCamera(settings);
        if (!camera) {
            qWarning() << "unable to load camera";
            return ZCalibratedCamera::Ptr(nullptr);
        }
    }
    settings->endGroup();

    /// camera calibration
    settings->beginGroup("CameraCalibration");
    {
        cameraCalibration = ZCameraCalibrationProvider::getCalibration(settings);
        if (!cameraCalibration) {
            qWarning() << "unable to load camera calibration";
            return ZCalibratedCamera::Ptr(nullptr);
        }
    }
    settings->endGroup();

    return ZCalibratedCamera::Ptr(new ZCalibratedCamera(camera, cameraCalibration));
}

} // namespace Z3D
