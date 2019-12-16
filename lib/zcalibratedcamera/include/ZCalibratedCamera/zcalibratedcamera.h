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

#include "zcalibratedcamera_global.h"

#include <Z3DCameraAcquisition>
#include <Z3DCameraCalibration>

namespace Z3D
{

class Z3D_CALIBRATEDCAMERA_SHARED_EXPORT ZCalibratedCamera : public QObject
{
    Q_OBJECT

public:
    explicit ZCalibratedCamera(ZCameraPtr camera, ZCameraCalibrationPtr cameraCalibration);
    ~ZCalibratedCamera();

    ZCameraPtr camera() const { return m_camera; }
    ZCameraCalibrationPtr calibration() const { return m_cameraCalibration; }

    void setCalibration(Z3D::ZCameraCalibrationPtr newCalibration);

signals:
    void calibrationChanged(Z3D::ZCameraCalibrationPtr newCalibration);

protected:
    ZCameraPtr m_camera;
    ZCameraCalibrationPtr m_cameraCalibration;
};

} // namespace Z3D
