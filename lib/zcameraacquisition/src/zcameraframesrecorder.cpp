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

#include "ZCameraAcquisition/zcameraframesrecorder.h"

#include "ZCameraAcquisition/zcameraimage.h"
#include "ZCameraAcquisition/zcamerainterface.h"

#include "ZCore/zlogging.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <opencv2/imgcodecs.hpp>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameraacquisition", QtInfoMsg)

namespace Z3D
{

ZCameraFramesRecorder::ZCameraFramesRecorder(const ZCameraWeakPtr &camera, QObject *parent)
    : QObject(parent)
    , m_camera(camera)
    , m_basePath(QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QLatin1String("acquisition")))
{
    if (!m_camera.isNull()) {
        zDebug() << "enabling frame recorder for camera:" << m_camera->uuid()
                 << "saving frames to:" << m_basePath;

        /// connect signals
        QObject::connect(m_camera.data(), &ZCameraInterface::acquisitionStarted,
                         this, &ZCameraFramesRecorder::onAcquisitionStarted);

        QObject::connect(m_camera.data(), &ZCameraInterface::newImageReceived,
                         this, &ZCameraFramesRecorder::onNewImageReceived);
    }
}

void ZCameraFramesRecorder::onAcquisitionStarted()
{
    /// update current save path
    const QDir dir(m_basePath);
    m_currentSavePath = dir.absoluteFilePath(
                QString("%1/%2")
                    .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss"))
                    .arg(m_camera->uuid())
    );

    if (!dir.mkpath(m_currentSavePath)) {
        zWarning() << "unable to create folder:" << m_currentSavePath;
    }
}

void ZCameraFramesRecorder::onNewImageReceived(const ZCameraImagePtr &image)
{
    QString currentImageFileName = QString("%1/%2.tiff")
            .arg(m_currentSavePath)
            .arg(image->number(), 8, 10, QLatin1Char('0'));

    cv::imwrite(qPrintable(currentImageFileName), image->cvMat());
}

} // namespace Z3D
