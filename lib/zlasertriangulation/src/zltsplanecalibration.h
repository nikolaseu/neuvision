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

#include "zlasertriangulation_global.h"
#include "zltscamera3dcalibrationinterface.h"

#include <Z3DCameraCalibration>

#include "opencv2/core/core.hpp"

namespace Z3D
{

class Z3D_LASERTRIANGULATION_SHARED_EXPORT LTSPlaneCalibration : public LTSCamera3DCalibrationInterface
{
public:
    LTSPlaneCalibration(Z3D::ZCameraCalibration::Ptr calibration);

    void getCalibratedPointForPixel(int x, int y, float *realX, float *realY, float *realZ);

    bool loadCalibration(const QString &fileName);

private:
    inline int indexForPixel(int x, int y) const { return x + y * m_sensorWidth; }

    void generateLookUpTable();

    /// camera calibration
    Z3D::ZCameraCalibration::Ptr m_cameraCalibration;

    /// sensor size (used to generate the look up table)
    int m_sensorWidth;
    int m_sensorHeight;

    /// distance to plane
    float m_planeDistance;

    /// plane normal vector
    float m_planeNormalX;
    float m_planeNormalY;
    float m_planeNormalZ;

    /// look up table
    std::vector<cv::Point3f> m_undistortedPoints;
};

} // namespace Z3D
