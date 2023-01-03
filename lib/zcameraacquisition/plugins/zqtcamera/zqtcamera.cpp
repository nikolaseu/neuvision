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

#include "zqtcamera.h"

#include "ZCameraAcquisition/zcameraimage.h"
#include "ZCore/zlogging.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <QtCore/QTimer>
#include <QtMultimedia/QCameraDevice>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameraacquisition.zqtcamera", QtInfoMsg)

namespace Z3D
{

ZQtCamera::ZQtCamera(QCamera *qcamera)
    : m_camera {qcamera}
    , m_captureSession {new QMediaCaptureSession(this)}
    , m_imageCapture {new QImageCapture(this)}
{
    m_captureSession->setCamera(m_camera);
    m_captureSession->setImageCapture(m_imageCapture);

    QObject::connect(m_imageCapture, &QImageCapture::imageAvailable,
                     this, &ZQtCamera::onImageAvailable);
    QObject::connect(m_imageCapture, &QImageCapture::readyForCaptureChanged,
                     this, &ZQtCamera::onReadyForCaptureChanged);
    QObject::connect(m_imageCapture, &QImageCapture::errorOccurred,
                     this, &ZQtCamera::onCaptureError);
}

ZQtCamera::~ZQtCamera()
{
    m_camera->stop();
    m_camera->deleteLater();
}

bool ZQtCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition() || m_isCapturing) {
        return false;
    }

    m_isCapturing = true;

    m_camera->start();

    return true;
}

bool ZQtCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition() || !m_isCapturing) {
        return false;
    }

    m_isCapturing = false;

    m_camera->stop();

    return true;
}

QList<ZCameraInterface::ZCameraAttribute> ZQtCamera::getAllAttributes()
{
    QList<ZCameraInterface::ZCameraAttribute> attrs;

    ZCameraInterface::ZCameraAttribute attrRes;
    attrRes.writable = attrRes.readable = true;
    attrRes.type = ZCameraInterface::CameraAttributeTypeEnum;
    attrRes.description = "Resolution";
    attrRes.id = "Resolution";
    attrRes.path = "Resolution";
    attrRes.label = "Resolution";
    const QList<QSize> supportedResolutions = m_camera->cameraDevice().photoResolutions();
    int currentIndex = 0;
    for(const QSize &resolution : supportedResolutions) {
        zInfo() << resolution;
        attrRes.enumNames << QString("%1x%2").arg(resolution.width()).arg(resolution.height());
        if (resolution == m_imageCapture->resolution()) {
            attrRes.enumValue = currentIndex;
        }
        currentIndex++;
    }
    attrs << attrRes;

    ZCameraInterface::ZCameraAttribute attrCodec;
    attrCodec.writable = attrCodec.readable = true;
    attrCodec.type = ZCameraInterface::CameraAttributeTypeEnum;
    attrCodec.description = "Image format";
    attrCodec.id = "Codec";
    attrCodec.path = "Codec";
    attrCodec.label = "Codec";
    currentIndex = 0;
    const QList<QImageCapture::FileFormat> supportedImageFormats = QImageCapture::supportedFormats();
    for (const QImageCapture::FileFormat &fileFormat : supportedImageFormats) {
        const QString codecName = QImageCapture::fileFormatName(fileFormat);
        attrCodec.enumNames << codecName;
        if (fileFormat == m_imageCapture->fileFormat()) {
            attrCodec.enumValue = currentIndex;
        }
        currentIndex++;
    }
    attrs << attrCodec;

    ZCameraInterface::ZCameraAttribute attrQuality;
    attrQuality.writable = attrQuality.readable = true;
    attrQuality.type = ZCameraInterface::CameraAttributeTypeEnum;
    attrQuality.description = "Quality";
    attrQuality.id = "Quality";
    attrQuality.path = "Quality";
    attrQuality.label = "Quality";
    static const QMetaEnum qualityEnum = QMetaEnum::fromType<QImageCapture::Quality>();
    for (int i=0; i<qualityEnum.keyCount(); ++i) {
        attrQuality.enumNames << qualityEnum.key(i);
        if (qualityEnum.value(i) == m_imageCapture->quality()) {
            attrQuality.enumValue = i;
        }
    }
    attrs << attrQuality;

    return attrs;
}

bool ZQtCamera::setAttribute(const QString &name, const QVariant &value)
{
    zDebug() << "trying to set" << name << "to" << value;

    if (name == "Resolution") {
        QStringList sizes = value.toString().split("x");
        const int width = sizes.front().toInt();
        const int height = sizes.back().toInt();
        const QSize resolution(width, height);
        m_imageCapture->setResolution(resolution);
        return m_imageCapture->resolution() == resolution;
    } else if (name == "Codec") {
        const QString codecName = value.toString();
        const QList<QImageCapture::FileFormat> supportedImageFormats = QImageCapture::supportedFormats();
        for (const QImageCapture::FileFormat &fileFormat : supportedImageFormats) {
            if (codecName == QImageCapture::fileFormatName(fileFormat)) {
                m_imageCapture->setFileFormat(fileFormat);
                return true;
            }
        }
    } else if (name == "Quality") {
        bool ok;
        const QString qualityStr = value.toString();
        static const QMetaEnum qualityEnum = QMetaEnum::fromType<QImageCapture::Quality>();
        const int qualityInt = qualityEnum.keyToValue(qualityStr.toLocal8Bit(), &ok);
        if (!ok) {
            return false;
        }
        const QImageCapture::Quality quality {static_cast<QImageCapture::Quality>(qualityInt)};
        m_imageCapture->setQuality(quality);
        return true;
    }

    return false;
}

QVariant ZQtCamera::getAttribute(const QString &name) const
{
    zDebug() << "trying to get" << name;
    //! TODO
    return false;
}

bool ZQtCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    zDebug() << "trying to set" << name << "to" << value << ", with notify:" << notify;
    //! TODO
    return false;
}

void ZQtCamera::onImageAvailable(int id, const QVideoFrame &buffer)
{
    const QImage preview = buffer.toImage();

    const auto &imgWidth = preview.width();
    const auto &imgHeight = preview.height();

    zDebug() << "imageAvailable:" << id << "size:" << QSize(imgWidth, imgHeight);

    cv::Mat cvImage8UC1;

    switch (preview.format()) {
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32: {
        cv::Mat cvImageOrig(imgHeight, imgWidth, CV_8UC4, (void *)preview.constBits(), preview.bytesPerLine());
        cv::cvtColor(cvImageOrig, cvImage8UC1, cv::COLOR_BGRA2GRAY);
        break;
    }
    default:
        zWarning() << "unknown format:" << preview.format();
    }

    if (!cvImage8UC1.empty()) {
        ZCameraImagePtr currentImage = this->getNextBufferImage(imgWidth, imgHeight, 0, 0, 1);

        currentImage->setBuffer((void *)cvImage8UC1.data);

        /// set image number
        currentImage->setNumber(id);

        /// notify
        emit newImageReceived(currentImage);
    }

    if (m_isCapturing) {
        QTimer::singleShot(0, m_imageCapture, [=](){
            m_imageCapture->capture();
        });
    }
}

void ZQtCamera::onCaptureError(int id, QImageCapture::Error error, const QString &message)
{
    zDebug() << "captureError:" << id << error << message;
    if (QImageCapture::NotReadyError == error && m_isCapturing) {
        QTimer::singleShot(0, m_imageCapture, [=](){
            m_imageCapture->capture();
        });
    }
}

void ZQtCamera::onReadyForCaptureChanged(bool ready)
{
    zDebug() << "readyForCaptureChanged:" << ready;

    if (ready) {
        QTimer::singleShot(0, m_imageCapture, [=](){
            m_imageCapture->capture();
        });
    }
}

} // namespace Z3D
