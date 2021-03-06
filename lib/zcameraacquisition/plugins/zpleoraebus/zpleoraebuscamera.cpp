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

#include "zpleoraebuscamera.h"

#include "zpleoraebuscamera_p.h"

#include "zcameraimage.h"

#include <QDebug>

namespace Z3D
{

ZCameraPtr ZPleoraeBUSCamera::getCameraByMAC(QString name)
{
    ZPleoraeBUSCameraPrivate *cameraPrivate = ZPleoraeBUSCameraPrivate::getCameraByMAC(name);

    if (!cameraPrivate)
        return nullptr;

    ZPleoraeBUSCamera *camera = new ZPleoraeBUSCamera();

    camera->d_ptr = cameraPrivate;
    camera->m_uuid = cameraPrivate->m_uuid;
    cameraPrivate->q_ptr = camera;

    QObject::connect(cameraPrivate, &ZPleoraeBUSCameraPrivate::newImageReceived,
                     camera,        &ZPleoraeBUSCamera::newImageReceived);
    QObject::connect(cameraPrivate, &ZPleoraeBUSCameraPrivate::error,
                     camera,        &ZPleoraeBUSCamera::error);
    QObject::connect(cameraPrivate, &ZPleoraeBUSCameraPrivate::warning,
                     camera,        &ZPleoraeBUSCamera::warning);
    QObject::connect(cameraPrivate, &ZPleoraeBUSCameraPrivate::attributeChanged,
                     camera,        &ZPleoraeBUSCamera::attributeChanged);

    return ZCameraPtr( camera );
}

ZPleoraeBUSCamera::ZPleoraeBUSCamera(QObject *parent)
    : ZCameraBase(parent)
{

}

ZPleoraeBUSCamera::~ZPleoraeBUSCamera()
{
    qDebug() << Q_FUNC_INFO;

    /// stop acquisition (if running)
    if (isRunning())
        stopAcquisition();

    d_ptr->deleteLater();
}

bool ZPleoraeBUSCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    return d_ptr->startAcquisition();
}

bool ZPleoraeBUSCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    return d_ptr->stopAcquisition();
}

QList<ZCameraInterface::ZCameraAttribute> ZPleoraeBUSCamera::getAllAttributes()
{
    return d_ptr->getAllAttributes();
}

QVariant ZPleoraeBUSCamera::getAttribute(const QString &name) const
{
    return d_ptr->getAttribute(name);
}

bool ZPleoraeBUSCamera::setBufferSize(int bufferSize)
{
    return ZCameraBase::setBufferSize(bufferSize)
            && d_ptr->setBufferSize(bufferSize);
}

bool ZPleoraeBUSCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    return d_ptr->setAttribute(name, value, notify);
}

} // namespace Z3D
