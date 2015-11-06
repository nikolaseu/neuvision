#include "zcameraframesrecorder.h"

#include "zcamerainterface.h"

//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>

namespace Z3D
{

ZCameraFramesRecorder::ZCameraFramesRecorder(QObject *parent) :
    QObject(parent),
    m_enabled(false)
{
    /// set default save path
    QDir dir(QCoreApplication::applicationDirPath());
    m_basePath = dir.absoluteFilePath( QLatin1String("acquisition") );

    qDebug() << "saving frames to:" << m_basePath;
}

ZCameraInterface::WeakPtr ZCameraFramesRecorder::getCamera() const
{
    return m_camera;
}

void ZCameraFramesRecorder::setCamera(ZCameraInterface::WeakPtr camera)
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

void ZCameraFramesRecorder::setEnabled(bool enabled)
{
    if (m_enabled != enabled) {
        if (enabled) {
            /// we are going to enable the recorder
            if (!m_camera.isNull()) {
                qDebug() << "enabling frame recorder for camera" << m_camera->uuid();

                /// connect signals
                QObject::connect(m_camera.data(), SIGNAL(acquisitionStarted()),
                                 this, SLOT(onAcquisitionStarted()));

                QObject::connect(m_camera.data(), SIGNAL(acquisitionStopped()),
                                 this, SLOT(onAcquisitionStopped()));

                QObject::connect(m_camera.data(), SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                                 this, SLOT(onNewImageReceived(Z3D::ZImageGrayscale::Ptr)));
            }
        } else {
            /// we are going to disable the recorder
            if (!m_camera.isNull()) {
                qDebug() << "disabling frame recorder for camera" << m_camera->uuid();

                /// disconnect signals (if camera was valid)
                QObject::disconnect(m_camera.data(), SIGNAL(acquisitionStarted()),
                                    this, SLOT(onAcquisitionStarted()));

                QObject::disconnect(m_camera.data(), SIGNAL(acquisitionStopped()),
                                    this, SLOT(onAcquisitionStopped()));

                QObject::disconnect(m_camera.data(), SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                                    this, SLOT(onNewImageReceived(Z3D::ZImageGrayscale::Ptr)));
            }
        }

        m_enabled = enabled;
    }
}

void ZCameraFramesRecorder::onAcquisitionStarted()
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

void ZCameraFramesRecorder::onAcquisitionStopped()
{
    qDebug() << Q_FUNC_INFO;
}

void ZCameraFramesRecorder::onNewImageReceived(Z3D::ZImageGrayscale::Ptr image)
{
    QString currentImageFileName = QString("%1/%2.tiff")
            .arg(m_currentSavePath)
            .arg(image->number(), 8, 10, QLatin1Char('0'));

//    qDebug() << "saving frame to:" << currentImageFileName;

    cv::imwrite(qPrintable(currentImageFileName), image->cvMat());
}

} // namespace Z3D
