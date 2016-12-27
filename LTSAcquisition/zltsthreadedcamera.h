/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ZLTSTHREADEDCAMERA_H
#define ZLTSTHREADEDCAMERA_H

#include <QObject>

#include <Z3DCameraAcquisition>
#include <Z3DLaserTriangulation>

class QThread;

class ZLTSThreadedCamera : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<ZLTSThreadedCamera> Ptr;

    explicit ZLTSThreadedCamera(Z3D::LTSCamera3D::Ptr camera3D, QObject *parent = 0);
    ~ZLTSThreadedCamera();

    Z3D::LTSCamera3D::Ptr camera3d() const;
    Z3D::ZCalibratedCamera::Ptr camera2d() const;

signals:

public slots:
    void startAcquisition();
    void stopAcquisition();

    void setFrameRecorderEnabled(bool enabled);
    void setCloudRecorderEnabled(bool enabled);

protected:
    Z3D::LTSCamera3D::Ptr m_camera3D;

    QThread *m_cameraThread;
    QThread *m_camera3DThread;

    Z3D::ZCameraFramesRecorder::Ptr m_framesRecorder;
    Z3D::PointCloudRecorder::Ptr m_cloudRecorder;
};

#endif // ZLTSTHREADEDCAMERA_H
