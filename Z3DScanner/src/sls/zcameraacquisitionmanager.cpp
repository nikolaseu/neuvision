#include "zcameraacquisitionmanager.h"

#include <QDir>

namespace Z3D {

ZCameraAcquisitionManager::ZCameraAcquisitionManager(std::vector<ZCameraInterface::Ptr> cameras, QObject *parent)
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
    std::vector<ZImageGrayscale::Ptr> images;
    images.reserve(m_cameras.size());

    for (auto cam : m_cameras) {
        /// if the camera is simulated, set which image should use now
        if (cam->uuid().startsWith("ZSIMULATED")) {
            qDebug() << "camera is simulated. using image" << id;
            cam->setAttribute("CurrentFile", id);
        }

        /// Retrieve an image
        ZImageGrayscale::Ptr imptr = cam->getSnapshot();

        if (imptr) {
            /// create a deep copy, the camera might return a temporary
            imptr = imptr->clone();

            if (m_debugMode) {
                /// save image
                imptr->save(QString("%1/%2/%3")
                            .arg(m_acquisitionId)
                            .arg(cam->uuid())
                            .arg(id));
            }
        } else {
            qCritical() << "error obtaining snapshot for camera" << cam->uuid();
        }

        images.push_back(imptr);
    }

    m_images.push_back(images);

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
