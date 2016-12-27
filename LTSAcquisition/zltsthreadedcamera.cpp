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

#include "zltsthreadedcamera.h"

#include <QThread>

ZLTSThreadedCamera::ZLTSThreadedCamera(Z3D::LTSCamera3D::Ptr camera3D, QObject *parent) :
    QObject(parent),
    m_camera3D(camera3D),
    m_cameraThread(new QThread(this)),
    m_camera3DThread(new QThread(this)),
    m_framesRecorder(new Z3D::ZCameraFramesRecorder()),
    m_cloudRecorder(new Z3D::PointCloudRecorder())
{
    m_camera3D->camera2d()->moveToThread(m_cameraThread);
    m_camera3D->moveToThread(m_camera3DThread);

    m_framesRecorder->setCamera(m_camera3D->camera2d()->camera().data());
    m_framesRecorder->moveToThread(m_camera3DThread);

    m_cloudRecorder->setCamera(m_camera3D.data());
    m_cloudRecorder->moveToThread(m_camera3DThread);

    /// scheduled as often as possible. more than highest
    m_cameraThread->start(QThread::TimeCriticalPriority);
    m_camera3DThread->start(QThread::HighestPriority);
}

ZLTSThreadedCamera::~ZLTSThreadedCamera()
{
    m_camera3D->stopAcquisition();

    m_cameraThread->quit();
    if (!m_cameraThread->wait(5000)) {
        m_cameraThread->terminate();
    }

    m_camera3DThread->quit();
    if (!m_camera3DThread->wait(5000)) {
        m_camera3DThread->terminate();
    }
}

Z3D::LTSCamera3D::Ptr ZLTSThreadedCamera::camera3d() const
{
    return m_camera3D;
}

Z3D::ZCalibratedCamera::Ptr ZLTSThreadedCamera::camera2d() const
{
    return m_camera3D->camera2d();
}

void ZLTSThreadedCamera::startAcquisition()
{
    m_camera3D->startAcquisition();
}

void ZLTSThreadedCamera::stopAcquisition()
{
    m_camera3D->stopAcquisition();
}

void ZLTSThreadedCamera::setFrameRecorderEnabled(bool enabled)
{
    m_framesRecorder->setEnabled(enabled);
}

void ZLTSThreadedCamera::setCloudRecorderEnabled(bool enabled)
{
    m_cloudRecorder->setEnabled(enabled);
}
