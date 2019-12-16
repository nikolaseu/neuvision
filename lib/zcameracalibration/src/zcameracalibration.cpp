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

#include "ZCameraCalibration/zcameracalibration.h"

#include <QMetaType>

#include <iostream>

#include <opencv2/core.hpp>

static int z3dCameraCalibrationPtrTypeId = qRegisterMetaType<Z3D::ZCameraCalibrationPtr>("Z3D::ZCameraCalibrationPtr");
static int z3dCameraCalibrationPtrVectorTypeId = qRegisterMetaType<std::vector<Z3D::ZCameraCalibrationPtr> >("std::vector<Z3D::ZCameraCalibrationPtr>");

namespace Z3D
{

ZCameraCalibration::ZCameraCalibration(CameraModelType cameraModelType)
    : m_sensorWidth(0)
    , m_sensorHeight(0)
    , m_cameraModelType(cameraModelType)
    , m_ready(false)
{
    /// no rotation => identity matrix
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    setRotation(rotation);

    /// no translation => zeros
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
    setTranslation(translation);

    //rotation.convertTo(m_rotation, CV_64F);
    //translation.convertTo(m_translation, CV_64F);
}



bool ZCameraCalibration::getCalibratedSubPixel(float x, float y, float *calibratedX, float *calibratedY)
{
    if (!m_ready)
        return false;

    /// we do a bilinear interpolation for subpixel values
    /// http://en.wikipedia.org/wiki/Bilinear_interpolation

    int x1 = int(floorf(x));
    int x2 = x1 + 1;
    int y1 = int(floorf(y));
    int y2 = y1 + 1;

    float cx11, cy11, cx12, cy12, cx21, cy21, cx22, cy22;

    /// if the subpixel is outside the calibrated range, getCalibratedPixel
    /// should return false, and so should this function
    if (!getCalibratedPixel(x1, y1, &cx11, &cy11)
            || !getCalibratedPixel(x1, y2, &cx12, &cy12)
            || !getCalibratedPixel(x2, y1, &cx21, &cy21)
            || !getCalibratedPixel(x2, y2, &cx22, &cy22))
        return false;

    /// first term is always one in our case
    /// 1 / ((x2-x1)*(y2-y1))

    const float x2_x = float(x2) - x;
    const float x_x1 = x - float(x1);
    const float y2_y = float(y2) - y;
    const float y_y1 = y - float(y1);

    const float mul_11 = x2_x * y2_y;
    const float mul_21 = x_x1 * y2_y;
    const float mul_12 = x2_x * y_y1;
    const float mul_22 = x_x1 * y_y1;

    *calibratedX = cx11 * mul_11 +
                   cx21 * mul_21 +
                   cx12 * mul_12 +
                   cx22 * mul_22;

    *calibratedY = cy11 * mul_11 +
                   cy21 * mul_21 +
                   cy12 * mul_12 +
                   cy22 * mul_22;

    return true;
}

bool ZCameraCalibration::getWorldRayForSubPixel(float x, float y, cv::Vec3d &origin, cv::Vec3d &direction)
{
    /// we do a bilinear interpolation for subpixel values
    /// http://en.wikipedia.org/wiki/Bilinear_interpolation

    float x1 = floorf(x);
    float x2 = x1 + 1;
    float y1 = floorf(y);
    float y2 = y1 + 1;

    /// check if the subpixel is inside the calibrated range
    if (x1 < 0 || x2 >= m_sensorWidth || y1 < 0 || y2 >= m_sensorHeight) {
        return getWorldRayForPixel(int(x1), int(y1), origin, direction);
    }

    cv::Vec3d origin11, direction11
            , origin12, direction12
            , origin21, direction21
            , origin22, direction22;

    getWorldRayForPixel(int(x1), int(y1), origin11, direction11);
    getWorldRayForPixel(int(x1), int(y2), origin12, direction12);
    getWorldRayForPixel(int(x2), int(y1), origin21, direction21);
    getWorldRayForPixel(int(x2), int(y2), origin22, direction22);

    /// first term is always one in our case
    /// 1 / ((x2-x1)*(y2-y1))

    if (m_cameraModelType == CentralCameraType) {
        /// if camera is central, all rays have the same origin
        origin = origin11;
    } else {
        origin = origin11 * (x2-x) * (y2-y) +
                 origin21 * (x-x1) * (y2-y) +
                 origin12 * (x2-x) * (y-y1) +
                 origin22 * (x-x1) * (y-y1);
    }

    direction = direction11 * (x2-x) * (y2-y) +
                direction21 * (x-x1) * (y2-y) +
                direction12 * (x2-x) * (y-y1) +
                direction22 * (x-x1) * (y-y1);

    return true;
}

bool ZCameraCalibration::setRotation(cv::Matx33d rotation)
{
    /// check validity
    cv::Matx33d I = cv::Matx33d::eye();
    cv::Matx33d I2 = rotation.t() * rotation;
    cv::Matx33d diff;
    cv::absdiff(I2, I, diff);
    /// allow some minor offset, in practice there is some rounding error
    if (cv::norm(diff) > 1e-15) {
        std::cout << "This should be the identity matrix, but has norm(R^T * R - I) = " << cv::norm(diff) << "\n"
                  << I2 << std::endl;
    }

    m_rotation = rotation;
    return true;
}

bool ZCameraCalibration::setTranslation(cv::Vec3d translation)
{
    m_translation = translation;
    return true;
}

void ZCameraCalibration::setReady(bool ready)
{
    if (m_ready == ready)
        return;

    m_ready = ready;

    emit calibrationReadyChanged(ready);

    if (ready)
        emit calibrationReady();
    else
        emit calibrationNotReady();
}

}
