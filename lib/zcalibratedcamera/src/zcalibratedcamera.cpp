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

#include "zcalibratedcamera.h"

#include <QDebug>
#include <QMetaType>

namespace Z3D
{

static int z3dCalibratedCameraListPtrTypeId = qRegisterMetaType< QList<Z3D::ZCalibratedCamera::Ptr> >("QList<Z3D::ZCalibratedCamera::Ptr>");

ZCalibratedCamera::ZCalibratedCamera(ZCameraInterface::Ptr camera, ZCameraCalibration::Ptr cameraCalibration) :
    m_camera(camera),
    m_cameraCalibration(cameraCalibration)
{

}

ZCalibratedCamera::~ZCalibratedCamera()
{
//    qDebug() << Q_FUNC_INFO;
}

void ZCalibratedCamera::setCalibration(ZCameraCalibration::Ptr newCalibration)
{
    if (m_cameraCalibration == newCalibration)
        return;

    m_cameraCalibration = newCalibration;

    emit calibrationChanged(newCalibration);
}

} // namespace Z3D
