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
    /// if no folder is indicated, use current running folder + "/cameras"
    if (folder.isEmpty()) {
        folder = QDir::current().absoluteFilePath("cameras");
    }

    QDir configDir(folder);

    qDebug() << "loading cameras from" << configDir.absolutePath();

    QList<Z3D::ZCalibratedCamera::Ptr> cameraList;

    QStringList filters;
    filters << "*.ini" << "*.json"; //! TODO: agregar para leer configuracion en JSON
    configDir.setNameFilters(filters);

    foreach (QString fileName, configDir.entryList(QDir::Files)) {
        qDebug() << "found" << fileName;
        QSettings settings(configDir.absoluteFilePath(fileName), QSettings::IniFormat);

        Z3D::ZCameraInterface::Ptr camera;
        Z3D::ZCameraCalibration::Ptr cameraCalibration;

        /// camera
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

        /// add to list
        cameraList << Z3D::ZCalibratedCamera::Ptr(new ZCalibratedCamera(camera, cameraCalibration));
    }

    return cameraList;
}

} // namespace Z3D
