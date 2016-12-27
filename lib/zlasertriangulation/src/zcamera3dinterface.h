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

#pragma once

#include <Z3DCalibratedCamera>
#include <Z3DCameraAcquisition>
#include <Z3DPointCloud>

#include <QObject>
#include <QPointer>
#include <QSharedPointer>

namespace Z3D
{

class Camera3DInterface : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<Z3D::Camera3DInterface> Ptr;
    typedef QPointer<Z3D::Camera3DInterface> WeakPtr;

    explicit Camera3DInterface(Z3D::ZCalibratedCamera::Ptr camera, QObject *parent = 0) :
        QObject(parent),
        m_camera(camera)
    {
        QObject::connect(m_camera->camera().data(), SIGNAL(acquisitionStarted()),
                         this, SIGNAL(acquisitionStarted()));
        QObject::connect(m_camera->camera().data(), SIGNAL(acquisitionStopped()),
                         this, SIGNAL(acquisitionStopped()));
    }

    QString uuid() { return m_camera->camera()->uuid(); }

signals:
    void newImageReceived(Z3D::ZImageGrayscale::Ptr image);
    void newPointCloudReceived(Z3D::ZPointCloud::Ptr cloud);

    void acquisitionStarted();
    void acquisitionStopped();

public slots:
    virtual void startAcquisition() = 0;
    virtual void stopAcquisition() = 0;

    Z3D::ZCalibratedCamera::Ptr camera2d() const { return m_camera; }

protected:
    Z3D::ZCalibratedCamera::Ptr m_camera;
};

} // namespace Z3D
