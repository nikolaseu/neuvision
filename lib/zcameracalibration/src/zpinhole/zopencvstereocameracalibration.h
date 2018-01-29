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

#include "zcameracalibration_global.h"
#include "zmulticameracalibration.h"

#include <opencv2/core/mat.hpp>

namespace Z3D
{

class Z3D_CAMERACALIBRATION_SHARED_EXPORT ZOpenCVStereoCameraCalibration : public ZMultiCameraCalibration
{
public:
    explicit ZOpenCVStereoCameraCalibration(const std::vector<ZCameraCalibrationPtr> &calibrations,
                                            const std::vector<std::vector<cv::Point3f> > &objectPoints,
                                            const std::vector<std::vector<std::vector<cv::Point2f> > > &imagePoints,
                                            const cv::Mat cameraMatrix[],
                                            const cv::Mat distCoeffs[],
                                            const cv::Size imageSize[],
                                            const cv::Mat &R,
                                            const cv::Mat &T,
                                            const cv::Mat &E,
                                            const cv::Mat &F);

    explicit ZOpenCVStereoCameraCalibration(const cv::Mat cameraMatrix[2],
                                            const cv::Mat distCoeffs[2],
                                            const cv::Size imageSize[2],
                                            const cv::Mat &R,
                                            const cv::Mat &T,
                                            const cv::Mat &E,
                                            const cv::Mat &F);

    const std::vector<std::vector<cv::Point3f>> objectPoints;
    const std::vector<std::vector<std::vector<cv::Point2f>>> imagePoints;
    std::vector<std::vector<std::vector<double>>> imagePointsEpipolarError;
    const cv::Mat cameraMatrix[2];
    const cv::Mat distCoeffs[2];
    const cv::Size imageSize[2];
    const cv::Mat R;
    const cv::Mat T;
    const cv::Mat E;
    const cv::Mat F;
};

} // namespace Z3D
