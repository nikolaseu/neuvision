#include "zqtcamera.h"

#include <QTimer>

namespace Z3D
{

ZQtCamera::ZQtCamera(QCamera *qcamera)
    : m_qcamera(qcamera)
{
    m_qcamera->setCaptureMode(QCamera::CaptureStillImage);
    //m_qcamera->setCaptureMode(QCamera::CaptureVideo);

    m_qcameraImageCapture = new QCameraImageCapture(m_qcamera);
    m_qcameraImageCapture->setCaptureDestination(QCameraImageCapture::CaptureToBuffer);
    m_qcameraImageCapture->setBufferFormat(QVideoFrame::Format_RGB24);
    //m_qcameraImageCapture->setBufferFormat(QVideoFrame::Format_CameraRaw);
    //m_qcameraImageCapture->setBufferFormat(QVideoFrame::Format_Y8);

    QObject::connect(m_qcameraImageCapture, &QCameraImageCapture::imageCaptured,
                     this, &ZQtCamera::onImageCaptured);
    QObject::connect(m_qcameraImageCapture, &QCameraImageCapture::imageAvailable,
                     this, &ZQtCamera::onImageAvailable);


    connect(m_qcameraImageCapture, SIGNAL(readyForCaptureChanged(bool)),
            this, SLOT(onReadyForCaptureChanged(bool)));
    //connect(m_qcameraImageCapture, SIGNAL(imageCaptured(int,QImage)),
    //        this, SLOT(processCapturedImage(int,QImage)));
    //connect(m_qcameraImageCapture, SIGNAL(imageSaved(int,QString)),
    //        this, SLOT(imageSaved(int,QString)));
    connect(m_qcameraImageCapture, SIGNAL(error(int,QCameraImageCapture::Error,QString)),
            this, SLOT(onCaptureError(int,QCameraImageCapture::Error,QString)));
}

ZQtCamera::~ZQtCamera()
{
    m_qcamera->deleteLater();
}

bool ZQtCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    m_qcamera->start();

    //on half pressed shutter button
    m_qcamera->searchAndLock();

    m_qcameraImageCapture->capture();

    return true;
}

bool ZQtCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    //on shutter button released
    m_qcamera->unlock();

    m_qcamera->stop();

    return true;
}

QList<ZCameraInterface::ZCameraAttribute> ZQtCamera::getAllAttributes()
{
    //! TODO
    QList<ZCameraInterface::ZCameraAttribute> attrs;
    return attrs;
}

bool ZQtCamera::setAttribute(const QString &name, const QVariant &value)
{
    //! TODO
    return false;
}

QVariant ZQtCamera::getAttribute(const QString &name) const
{
    //! TODO
    return false;
}

bool ZQtCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    //! TODO
    return false;
}

void ZQtCamera::onImageCaptured(int id, const QImage &preview)
{
    //qDebug() << "imageCaptured:" << id << "format:" << preview.format();

    switch (preview.format()) {
    case QVideoFrame::Format_RGB24: {
        /// 4
        /// QVideoFrame::Format_RGB24
        /// The frame is stored using a 24-bit RGB format (8-8-8). This is equivalent to QImage::Format_RGB888
        int imgWidth = preview.width()
          , imgHeight = preview.height();

        /// QImage::constBits()
        /// Returns a pointer to the first pixel data.
        /// Note that QImage uses implicit data sharing, but this function does
        /// not perform a deep copy of the shared pixel data, because the
        /// returned data is const.
        cv::Mat cvImageRGB888(imgHeight, imgWidth, CV_8UC4, (void *)preview.constBits());

        cv::Mat cvImage8UC1;
        cv::cvtColor(cvImageRGB888, cvImage8UC1, cv::COLOR_BGRA2GRAY);

        ZImageGrayscale::Ptr currentImage = this->getNextBufferImage(
                    imgWidth,
                    imgHeight,
                    0,
                    0,
                    1);

        currentImage->setBuffer((void *)cvImage8UC1.data);

        /// set image number
        currentImage->setNumber(id);

        /// notify
        emit newImageReceived(currentImage);
        break;
    }
    case QVideoFrame::Format_AYUV444_Premultiplied: {
        /// 16
        /// QVideoFrame::Format_AYUV444_Premultiplied
        /// The frame is stored using a packed premultiplied 32-bit AYUV format (0xAAYYUUVV).
        int imgWidth = preview.width()
          , imgHeight = preview.height();

        /// QImage::constBits()
        /// Returns a pointer to the first pixel data.
        /// Note that QImage uses implicit data sharing, but this function does
        /// not perform a deep copy of the shared pixel data, because the
        /// returned data is const.
        cv::Mat cvImageRGB888(imgHeight, imgWidth, CV_8UC4, (void *)preview.constBits());

        cv::Mat cvImage8UC1;
        cv::cvtColor(cvImageRGB888, cvImage8UC1, cv::COLOR_BGRA2GRAY);

        ZImageGrayscale::Ptr currentImage = this->getNextBufferImage(
                    imgWidth,
                    imgHeight,
                    0,
                    0,
                    1);

        currentImage->setBuffer((void *)cvImage8UC1.data);

        /// set image number
        currentImage->setNumber(id);

        /// notify
        emit newImageReceived(currentImage);
        break;
    }
    default:
        qWarning() << "unknown format:" << preview.format();
    }

    if (m_qcamera->state() == QCamera::ActiveState)
        QTimer::singleShot(0, m_qcameraImageCapture, SLOT(capture()));
}

void ZQtCamera::onImageAvailable(int id, const QVideoFrame &buffer)
{
    //qDebug() << "imageAvailable:" << id;
}

void ZQtCamera::onCaptureError(int id, QCameraImageCapture::Error error, QString message)
{
    qDebug() << "captureError:" << id << error << message;
    //QTimer::singleShot(100, m_qcameraImageCapture, SLOT(capture()));
}

void ZQtCamera::onReadyForCaptureChanged(bool ready)
{
    qDebug() << "readyForCaptureChanged:" << ready;
    QTimer::singleShot(0, m_qcameraImageCapture, SLOT(capture()));
}

} // namespace Z3D
