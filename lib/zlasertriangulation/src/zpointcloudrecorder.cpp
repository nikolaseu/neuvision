#include "zpointcloudrecorder.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>

#include "pcl/io/pcd_io.h"

namespace Z3D
{

PointCloudRecorder::PointCloudRecorder(QObject *parent) :
    QObject(parent),
    m_enabled(false)
{
    /// set default save path
    QDir dir(QCoreApplication::applicationDirPath());
    m_basePath = dir.absoluteFilePath( QLatin1String("acquisition_3d") );

    qDebug() << "saving point clouds to:" << m_basePath;
}

Camera3DInterface::WeakPtr PointCloudRecorder::getCamera() const
{
    return m_camera;
}

void PointCloudRecorder::setCamera(Camera3DInterface::WeakPtr camera)
{
    if (m_camera.isNull() || m_camera != camera) {
        bool wasEnabled = m_enabled;

        /// force disconnection from old camera signals
        setEnabled(false);

        /// update camera shared pointer
        m_camera = camera;

        /// only enable if was enabled previously
        setEnabled(wasEnabled);
    }
}

void PointCloudRecorder::setEnabled(bool enabled)
{
    if (m_enabled != enabled) {
        if (enabled) {
            /// we are going to enable the recorder
            if (!m_camera.isNull()) {
                qDebug() << "enabling point cloud recorder for camera" << m_camera->uuid();

                /// connect signals
                QObject::connect(m_camera.data(), SIGNAL(acquisitionStarted()),
                                 this, SLOT(onAcquisitionStarted()));

                QObject::connect(m_camera.data(), SIGNAL(acquisitionStopped()),
                                 this, SLOT(onAcquisitionStopped()));

                QObject::connect(m_camera.data(), SIGNAL(newPointCloudReceived(Z3D::ZPointCloud::Ptr)),
                                 this, SLOT(onNewPointCloudReceived(Z3D::ZPointCloud::Ptr)));
            }
        } else {
            /// we are going to disable the recorder
            if (!m_camera.isNull()) {
                qDebug() << "disabling point cloud recorder for camera" << m_camera->uuid();

                /// disconnect signals (if camera was valid)
                QObject::disconnect(m_camera.data(), SIGNAL(acquisitionStarted()),
                                    this, SLOT(onAcquisitionStarted()));

                QObject::disconnect(m_camera.data(), SIGNAL(acquisitionStopped()),
                                    this, SLOT(onAcquisitionStopped()));

                QObject::disconnect(m_camera.data(), SIGNAL(newPointCloudReceived(Z3D::ZPointCloud::Ptr)),
                                    this, SLOT(onNewPointCloudReceived(Z3D::ZPointCloud::Ptr)));
            }
        }

        m_enabled = enabled;
    }
}

void PointCloudRecorder::onAcquisitionStarted()
{
    /// update current save path
    QDir dir( m_basePath );
    m_currentSavePath = dir.absoluteFilePath(
                QString("%1/%2")
                    .arg( QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss") )
                    .arg( m_camera->uuid() )
    );

    if (!dir.mkpath(m_currentSavePath)) {
        qWarning() << Q_FUNC_INFO << "unable to create folder:" << m_currentSavePath;
    }
}

void PointCloudRecorder::onAcquisitionStopped()
{
    qDebug() << Q_FUNC_INFO;
}

void PointCloudRecorder::onNewPointCloudReceived(Z3D::ZPointCloud::Ptr cloud)
{
    QString currentImageFileName = QString("%1/%2.pcd")
            .arg(m_currentSavePath)
            .arg(cloud->number(), 8, 10, QLatin1Char('0'));

//    qDebug() << "saving frame to:" << currentImageFileName;

    pcl::io::savePCDFile<PointType>(qPrintable(currentImageFileName), *cloud->pclPointCloud(), true);
    //pcl::io::savePCDFileBinary<PointType>(qPrintable(currentImageFileName), *cloud->pclPointCloud());
    //cv::imwrite(qPrintable(currentImageFileName), Z3D::ImageUtils::toCVMat(image));
}

} // namespace Z3D
