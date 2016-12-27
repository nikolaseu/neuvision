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

#include "zniimaqdxgrabcamera.h"

#include "zniimaqdxgrabcamera_p.h"

#include "zcameraimage.h"

#include <QDebug>

namespace Z3D
{

ZCameraInterface::Ptr ZNIIMAQdxGrabCamera::getCameraByName(QString name)
{
    ZNIIMAQdxGrabCameraPrivate *cameraPrivate = ZNIIMAQdxGrabCameraPrivate::getCameraByName(name);

    if (!cameraPrivate)
        return ZCameraInterface::Ptr(0);

    ZNIIMAQdxGrabCamera *camera = new ZNIIMAQdxGrabCamera();

    camera->d_ptr = cameraPrivate;
    camera->m_uuid = cameraPrivate->m_uuid;
    cameraPrivate->q_ptr = camera;

    QObject::connect(cameraPrivate, SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                     camera,        SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)));
    QObject::connect(cameraPrivate, SIGNAL(error(QString)),
                     camera,        SIGNAL(error(QString)));
    QObject::connect(cameraPrivate, SIGNAL(warning(QString)),
                     camera,        SIGNAL(warning(QString)));
    QObject::connect(cameraPrivate, SIGNAL(attributeChanged(QString,QVariant)),
                     camera,        SIGNAL(attributeChanged(QString,QVariant)));

    return ZCameraInterface::Ptr( camera );
}

ZNIIMAQdxGrabCamera::ZNIIMAQdxGrabCamera(QObject *parent) :
    ZCameraBase(parent)
{

}

ZNIIMAQdxGrabCamera::~ZNIIMAQdxGrabCamera()
{
    /// stop acquisition (if running)
    if (isRunning())
        stopAcquisition();

    d_ptr->deleteLater();
}

bool ZNIIMAQdxGrabCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    return d_ptr->startAcquisition();
}

bool ZNIIMAQdxGrabCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    return d_ptr->stopAcquisition();
}

QList<ZCameraInterface::ZCameraAttribute> ZNIIMAQdxGrabCamera::getAllAttributes()
{
    return d_ptr->getAllAttributes();
}

QVariant ZNIIMAQdxGrabCamera::getAttribute(const QString &name) const
{
    return d_ptr->getAttribute(name);
}

bool ZNIIMAQdxGrabCamera::setBufferSize(int bufferSize)
{
    return d_ptr->setBufferSize(bufferSize);
}

bool ZNIIMAQdxGrabCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    return d_ptr->setAttribute(name, value, notify);
}

} // namespace Z3D
