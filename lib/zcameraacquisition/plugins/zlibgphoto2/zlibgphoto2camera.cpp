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

#include "zlibgphoto2camera.h"

#include <QDebug>
#include <QSize>
#include <QThread>
#include <QTimer>


namespace Z3D
{

ZLibGPhoto2Camera::ZLibGPhoto2Camera(QObject *parent) :
    ZCameraBase(parent)
{

}

ZLibGPhoto2Camera::~ZLibGPhoto2Camera()
{

}

bool ZLibGPhoto2Camera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    return true;
}

bool ZLibGPhoto2Camera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    return true;
}

QList<ZCameraInterface::ZCameraAttribute> ZLibGPhoto2Camera::getAllAttributes()
{
    QList<ZCameraInterface::ZCameraAttribute> attributeList;

    return attributeList;
}

QVariant ZLibGPhoto2Camera::getAttribute(const QString &/*name*/) const
{
    //! TODO
    return QVariant("INVALID");
}

bool ZLibGPhoto2Camera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    bool changed = false;

    /// now the mutex is released, we could update settings in case something changed
    if (notify) {
        emit attributeChanged("","");
    }

    return changed;
}

} // namespace Z3D
