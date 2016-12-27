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

#ifndef Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATION_H
#define Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATION_H

#include "cameracalibrationinterface.h"
#include "opencv2/core/core.hpp"

namespace Z3D
{

class PinholeCameraCalibration : public CameraCalibrationInterface
{
public:
    PinholeCameraCalibration();

    virtual void getCalibratedPixel(int x, int y,
                                    float *calibratedX, float *calibratedY);

    virtual void getRayForPixel(int x, int y, /// pixel coordinates
                                float *origin, /// ray origin
                                float *direction); /// ray direction

    virtual bool loadCalibration(const QString &fileName);

protected:
    inline int indexForPixel(int x, int y) const { return x + y * m_sensorWidth; }

    void generateLookUpTable();

    cv::Matx33f m_cvRotation;
    cv::Vec3f m_cvTranslation;

    cv::Mat m_cvCameraMatrix;
    cv::Mat m_cvDistortionCoeffs;

    std::vector<cv::Point2f> m_undistortedPoints;
    //std::vector<cv::Vec3f> m_undistortedRays;
    //std::vector<cv::Vec3f> m_undistortedWorldRays;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATION_H
