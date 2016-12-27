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
