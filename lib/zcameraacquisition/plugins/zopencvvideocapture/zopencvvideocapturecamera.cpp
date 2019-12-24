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

#include "zopencvvideocapturecamera.h"

#include "ZCameraAcquisition/zcameraimage.h"

#include <opencv2/imgproc.hpp>

#include <QDebug>
#include <QSize>
#include <QThread>
#include <QTimer>

#define ATTR_MODE "Mode"
#define ATTR_MODE_CUSTOM "CUSTOM"

namespace Z3D
{

// static var
QMap<int, QString> OpenCVVideoCaptureCamera::m_opencvAttributeNames;

OpenCVVideoCaptureCamera::OpenCVVideoCaptureCamera(cv::VideoCapture *videoCapture, QObject *parent)
    : ZCameraBase(parent)
    , m_capture(videoCapture)
{
    m_uuid = QString("ZOpenCV-%1").arg(long(videoCapture));

    if (m_opencvAttributeNames.empty()) {
        /// attribute id <-> name map
        m_opencvAttributeNames[cv::CAP_PROP_FRAME_WIDTH ] = "OpenCV::Frame::Width";
        m_opencvAttributeNames[cv::CAP_PROP_FRAME_HEIGHT] = "OpenCV::Frame::Height";
        m_opencvAttributeNames[cv::CAP_PROP_BRIGHTNESS  ] = "OpenCV::Brightness";
        m_opencvAttributeNames[cv::CAP_PROP_CONTRAST    ] = "OpenCV::Contrast";
        m_opencvAttributeNames[cv::CAP_PROP_SATURATION  ] = "OpenCV::Saturation";
        m_opencvAttributeNames[cv::CAP_PROP_HUE         ] = "OpenCV::Hue";
        m_opencvAttributeNames[cv::CAP_PROP_GAIN        ] = "OpenCV::Gain";
        m_opencvAttributeNames[cv::CAP_PROP_EXPOSURE    ] = "OpenCV::Exposure";
        m_opencvAttributeNames[cv::CAP_PROP_FPS         ] = "OpenCV::FPS";
    }

    /*/// try to get supported frame sizes
    union {double prop; const char* name;} u;
    u.prop = m_capture->get(cv::CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING);
    // VideoCapture::get can return 0.0 or -1.0 if it doesn't support
    // cv::CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING
    if (u.prop != 0.0 && u.prop != -1.0) {
        qDebug() << "cv::CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING" << u.name;
    } else */{
        m_frameSizes << QSize(320, 240)
                     << QSize(640, 480)
                     << QSize(640, 488)
                     << QSize(800, 600)
                     << QSize(848, 480)
                     << QSize(1280, 720)
                     << QSize(1280, 1024)
                     << QSize(1600, 1200);
    }

    /// create a thread for the camera and move the camera to it
    auto *cameraThread = new QThread();
    qDebug() << qPrintable(
                    QString("[%1] moving camera to its own thread (0x%2)")
                    .arg(this->uuid())
                    .arg(long(cameraThread), 0, 16));
    this->moveToThread(cameraThread);
    /// start thread with the highest priority
    cameraThread->start(QThread::HighPriority);

    ///
    qDebug() << Q_FUNC_INFO << uuid();
}

OpenCVVideoCaptureCamera::~OpenCVVideoCaptureCamera()
{
    if (isRunning()) {
        /// thread safe
        QMutexLocker locker(&m_mutex);

        stopAcquisition();
    }

    qDebug() << Q_FUNC_INFO << uuid();

    delete m_capture;
    m_capture = nullptr;
}

bool OpenCVVideoCaptureCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    qDebug() << Q_FUNC_INFO;

    /// start running grab loop in the camera's thread
    m_stopThreadRequested = false;
    QTimer::singleShot(0, this, &OpenCVVideoCaptureCamera::grabLoop);

    return true;
}

bool OpenCVVideoCaptureCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    qDebug() << Q_FUNC_INFO;

    m_stopThreadRequested = true;

    return true;
}

QList<ZCameraInterface::ZCameraAttribute> OpenCVVideoCaptureCamera::getAllAttributes()
{
    /// thread safe
    QMutexLocker locker(&m_mutex);

    QList<ZCameraInterface::ZCameraAttribute> attributeList;

    /*
    for (int i=0; i<1100; ++i) {
        double v = m_capture->get(i);
        if (v != -1.)
            qDebug() << i << v;
    }

    union {double prop; const char* name;} u;
    u.prop = m_capture->get(cv::CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING);
    // VideoCapture::get can return 0.0 or -1.0 if it doesn't support
    // cv::CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING
    if (u.prop != 0.0 && u.prop != -1.0)
        qDebug() << "cv::CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING" << u.name;
    */

    int currentWidth = (int) m_capture->get( cv::CAP_PROP_FRAME_WIDTH );
    int currentHeight = (int) m_capture->get( cv::CAP_PROP_FRAME_HEIGHT );

    ZCameraInterface::ZCameraAttribute mode;
    mode.type = ZCameraInterface::CameraAttributeTypeEnum;
    mode.id = ATTR_MODE;
    mode.path = ATTR_MODE;
    mode.label = ATTR_MODE;
    for (int is = 0; is<m_frameSizes.size(); ++is) {
        const QSize &size = m_frameSizes[is];
        mode.enumNames << QString("%1x%2").arg(size.width()).arg(size.height());
        if (currentWidth == size.width() && currentHeight == size.height())
            mode.enumValue = is;
    }
    /// if mode unknown
    if (mode.enumValue < 0) {
        mode.enumNames << ATTR_MODE_CUSTOM;
        mode.enumValue = mode.enumNames.size() - 1;
    }
    mode.readable = true;
    mode.writable = true;
    attributeList << mode;

    for (int key : m_opencvAttributeNames.keys()) {
        ZCameraInterface::ZCameraAttribute attr;
        if (key == cv::CAP_PROP_FRAME_WIDTH || key == cv::CAP_PROP_FRAME_HEIGHT) {
            attr.type = ZCameraInterface::CameraAttributeTypeInt;
            attr.minimumValue = -999999;
            attr.maximumValue = 999999;
        } else {
            attr.type = ZCameraInterface::CameraAttributeTypeFloat;
            attr.minimumValue = -DBL_MAX;
            attr.maximumValue = DBL_MAX;
        }
        const auto &name = m_opencvAttributeNames[key];
        attr.id = name;
        attr.path = name;
        attr.label = name.mid(name.lastIndexOf("::") + 2);
        attr.value = m_capture->get(key);
        attr.readable = true;
        attr.writable = true;

        attributeList << attr;
    }

    return attributeList;
}

QVariant OpenCVVideoCaptureCamera::getAttribute(const QString &/*name*/) const
{
    //! TODO
    return QVariant("INVALID");
}

bool OpenCVVideoCaptureCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    bool changed = false;

    /// mutex locked only while modifying settings, release as soon as possible
    /// to avoid dead lock!
    {
        /// thread safe
        QMutexLocker locker(&m_mutex);

        qDebug() << "setting attribute" << name << "to" << value;

        if (name == ATTR_MODE) {
            QString mode = value.toString();
            if (mode != ATTR_MODE_CUSTOM) {
                QStringList size = mode.split("x");
                int width = size[0].toInt();
                int height = size[1].toInt();

                /// set values
                if (m_capture->get( cv::CAP_PROP_FRAME_WIDTH) != width || m_capture->get( cv::CAP_PROP_FRAME_HEIGHT) != height) {
                    /// set both together, this values doesn't change independently,
                    /// they are valid only in certain combinations supported by
                    /// the device
                    m_capture->set( cv::CAP_PROP_FRAME_WIDTH, width );
                    m_capture->set( cv::CAP_PROP_FRAME_HEIGHT, height );

                    /// check if both were configured correctly
                    changed = m_capture->get( cv::CAP_PROP_FRAME_WIDTH) == width
                            && m_capture->get( cv::CAP_PROP_FRAME_HEIGHT) == height;
                }

                if (!changed)
                    qWarning() << "unable to change resolution to" << width << "x" << height;
            }
        } else if (-666 != m_opencvAttributeNames.key(name, -666)) {
            int key = m_opencvAttributeNames.key(name);
            int doubleValue(value.toDouble());
            if (m_capture->get( key) != doubleValue) {
                m_capture->set( key, doubleValue );
                changed = m_capture->get( key ) == doubleValue;
            }
        } else {
            qWarning() << "unknown attribute" << name;
        }
    }

    /// now the mutex is released, we could update settings in case something changed
    if (notify)
        emit attributeChanged("","");

    return changed;
}

void OpenCVVideoCaptureCamera::grabLoop()
{
    cv::Mat frame;
    int bytesPerPixel;
    int frameCounter = -1;

    while (!m_stopThreadRequested && m_capture) {
        {
            /// thread safe
//            QMutexLocker locker(&m_mutex);
            if (!m_mutex.tryLock(100)) {
                continue;
            }

            /// grab & retrieve next frame
            m_capture->read(frame);

            m_mutex.unlock();
        }

        if (frame.empty()) {
            qWarning() << "error getting frame. frame is empty!";
            break;
        }

        qDebug() << "acquired image" << frameCounter
                 << "at" << m_capture->get(cv::CAP_PROP_FPS) << "FPS"
                 << "pos frames:" << m_capture->get(cv::CAP_PROP_POS_FRAMES)
                 << "pos msec:" << m_capture->get(cv::CAP_PROP_POS_MSEC);

        switch (frame.type()) {
        case CV_8UC1:
            bytesPerPixel = 1;
            break;
        case CV_16UC1:
            bytesPerPixel = 2;
            break;
        default:
            bytesPerPixel = 1;
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            frame.convertTo(frame, CV_8UC1);
        }

        ZCameraImagePtr currentImage = getNextBufferImage(frame.cols, frame.rows,
                                                             0, 0,
                                                             bytesPerPixel);

        /// copy data
        currentImage->setBuffer(frame.data);

        /// set image number
        currentImage->setNumber(++frameCounter);

        //qDebug() << "frame" << frameCounter;

        /// notify
        emit newImageReceived(currentImage);
    }

    qDebug() << "acquired" << frameCounter << "images";

    if (!m_stopThreadRequested) {
        /// if we stopped because of some error, notify
        stopAcquisition();
    }
}

} // namespace Z3D
