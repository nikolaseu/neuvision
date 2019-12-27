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

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <QCameraViewfinder>
#include <QDir>
#include <QFile>
#include <QTimer>

namespace Z3D
{

ZQtCamera::ZQtCamera(QCamera *qcamera)
    : m_qcamera(qcamera)
    , m_isCapturing(false)
{
    m_qcamera->setCaptureMode(QCamera::CaptureStillImage);
    //m_qcamera->setCaptureMode(QCamera::CaptureVideo);

    m_qcameraImageCapture = new QCameraImageCapture(m_qcamera);
    if (m_qcameraImageCapture->isCaptureDestinationSupported(QCameraImageCapture::CaptureToBuffer)) {
        m_qcameraImageCapture->setCaptureDestination(QCameraImageCapture::CaptureToBuffer);
    } else {
        qWarning() << "QCameraImageCapture::CaptureToBuffer not supported, will save capture to files :(";
    }
    m_qcameraImageCapture->setBufferFormat(QVideoFrame::Format_RGB24);
    //m_qcameraImageCapture->setBufferFormat(QVideoFrame::Format_CameraRaw);
    //m_qcameraImageCapture->setBufferFormat(QVideoFrame::Format_Y8);

    connect(m_qcameraImageCapture, &QCameraImageCapture::readyForCaptureChanged,
            this, &ZQtCamera::onReadyForCaptureChanged);
    connect(m_qcameraImageCapture, &QCameraImageCapture::imageCaptured,
            this, &ZQtCamera::onImageCaptured);
    connect(m_qcameraImageCapture, &QCameraImageCapture::imageAvailable,
            this, &ZQtCamera::onImageAvailable);
    connect(m_qcameraImageCapture, &QCameraImageCapture::imageSaved,
            this, &ZQtCamera::onImageSaved);
    connect(m_qcameraImageCapture, static_cast<void(QCameraImageCapture::*)(int, QCameraImageCapture::Error, const QString &)>(&QCameraImageCapture::error),
            this, &ZQtCamera::onCaptureError);

    auto *viewfinder = new QCameraViewfinder();
    //viewfinder->show();
    m_qcamera->setViewfinder(viewfinder);
    m_qcamera->start();
}

ZQtCamera::~ZQtCamera()
{
    m_qcamera->stop();
    m_qcamera->deleteLater();
}

bool ZQtCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition() || m_isCapturing)
        return false;

    m_isCapturing = true;

    m_qcameraImageCapture->capture();

    return true;
}

bool ZQtCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition() || !m_isCapturing)
        return false;

    m_isCapturing = false;

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
    QList<QSize> supportedResolutions = m_qcameraImageCapture->supportedResolutions();
    int currentIndex = 0;
    for(const QSize &resolution : supportedResolutions) {
        attrRes.enumNames << QString("%1x%2").arg(resolution.width()).arg(resolution.height());
        if (resolution == m_qcameraImageCapture->encodingSettings().resolution()) {
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
    for (const QString &codecName : m_qcameraImageCapture->supportedImageCodecs()) {
        attrCodec.enumNames << codecName;
        if (codecName == m_qcameraImageCapture->encodingSettings().codec()) {
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
    attrQuality.enumNames << "VeryLowQuality"
                          << "LowQuality"
                          << "NormalQuality"
                          << "HighQuality"
                          << "VeryHighQuality";
    attrQuality.enumValue = m_qcameraImageCapture->encodingSettings().quality();
    attrs << attrQuality;

    return attrs;
}

bool ZQtCamera::setAttribute(const QString &name, const QVariant &value)
{
    qDebug() << "trying to set" << name << "to" << value;
    QImageEncoderSettings settings = m_qcameraImageCapture->encodingSettings();
    if (name == "Resolution") {
        QStringList sizes = value.toString().split("x");
        const int width = sizes.front().toInt();
        const int height = sizes.back().toInt();
        settings.setResolution(width, height);
    } else if (name == "Codec") {
        settings.setCodec(value.toString());
    } else if (name == "Quality") {
        QString quality = value.toString();
        if (quality == "VeryLowQuality") {
            settings.setQuality(QMultimedia::VeryLowQuality);
        } else if (quality == "LowQuality") {
            settings.setQuality(QMultimedia::LowQuality);
        } else if (quality == "NormalQuality") {
            settings.setQuality(QMultimedia::NormalQuality);
        } else if (quality == "HighQuality") {
            settings.setQuality(QMultimedia::HighQuality);
        } else if (quality == "VeryHighQuality") {
            settings.setQuality(QMultimedia::VeryHighQuality);
        }
    }
    m_qcameraImageCapture->setEncodingSettings(settings);

    return false;
}

QVariant ZQtCamera::getAttribute(const QString &name) const
{
    qDebug() << "trying to get" << name;
    //! TODO
    return false;
}

bool ZQtCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    qDebug() << "trying to set" << name << "to" << value << ", with notify:" << notify;
    //! TODO
    return false;
}

void ZQtCamera::onImageCaptured(int id, const QImage &preview)
{
    //qDebug() << "imageCaptured:" << id << "format:" << preview.format();

    const auto &imgWidth = preview.width();
    const auto &imgHeight = preview.height();

    cv::Mat cvImage8UC1;
    cv::Mat cvImageOrig;

    switch (preview.format()) {
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32:
        cvImageOrig = cv::Mat (imgHeight, imgWidth, CV_8UC4, (void *)preview.constBits(), preview.bytesPerLine());
        cv::cvtColor(cvImageOrig, cvImage8UC1, cv::COLOR_BGRA2GRAY);
        break;
    default:
        qWarning() << "unknown format:" << preview.format();
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
        QTimer::singleShot(0, [=](){
            m_qcameraImageCapture->capture();
        });
    }
}

void ZQtCamera::onImageSaved(int id, const QString &fileName)
{
    if (QFile::remove(fileName)) {
        qDebug() << "imageSaved and erased:" << id << fileName;
    } else {
        qDebug() << "imageSaved:" << id << fileName;
    }
}

void ZQtCamera::onImageAvailable(int id, const QVideoFrame &buffer)
{
    Q_UNUSED(buffer)
    qDebug() << "imageAvailable:" << id;
}

void ZQtCamera::onCaptureError(int id, QCameraImageCapture::Error error, const QString &message)
{
    qDebug() << "captureError:" << id << error << message;
    if (QCameraImageCapture::NotReadyError == error && m_isCapturing) {
        QTimer::singleShot(100, [=](){
            m_qcameraImageCapture->capture();
        });
    }
}

void ZQtCamera::onReadyForCaptureChanged(bool ready)
{
    qDebug() << "readyForCaptureChanged:" << ready;
}

} // namespace Z3D
