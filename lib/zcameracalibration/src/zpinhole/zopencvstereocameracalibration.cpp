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

#include "zopencvstereocameracalibration.h"
#include "zpinholecameracalibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <QDebug>

namespace Z3D
{

ZOpenCVStereoCameraCalibration::ZOpenCVStereoCameraCalibration(const std::vector<ZCameraCalibrationPtr> &calibrations,
                                                               const std::vector<std::vector<cv::Point3f> > &objectPoints,
                                                               const std::vector<std::vector<std::vector<cv::Point2f> > > &imagePoints,
                                                               const cv::Mat cameraMatrix[],
                                                               const cv::Mat distCoeffs[],
                                                               const cv::Size imageSize[],
                                                               const cv::Mat &R,
                                                               const cv::Mat &T,
                                                               const cv::Mat &E,
                                                               const cv::Mat &F)
    : ZMultiCameraCalibration(calibrations)
    , objectPoints(objectPoints)
    , imagePoints(imagePoints)
    , cameraMatrix { cameraMatrix[0], cameraMatrix[1] }
    , distCoeffs { distCoeffs[0], distCoeffs[1] }
    , imageSize { imageSize[0], imageSize[1] }
    , R(R)
    , T(T)
    , E(E)
    , F(F)
{
    const auto ncameras = 2;
    const auto nimages = imagePoints.empty() ? 0 : imagePoints[0].size();
    imagePointsEpipolarError.resize(ncameras);
    for (size_t cam=0; cam<ncameras; ++cam) {
        imagePointsEpipolarError[cam].resize(nimages);
        for (size_t image=0; image<nimages; ++image) {
            const int npoints = imagePoints[cam][image].size();
            imagePointsEpipolarError[cam][image].resize(npoints);
        }
    }

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for (size_t i = 0; i < nimages; i++ ) {
        const auto npt = imagePoints[0][i].size();
        cv::Mat imgpt[2];

        for (size_t k = 0; k < 2; k++ ) {
            imgpt[k] = cv::Mat(imagePoints[k][i]);
            cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
            cv::computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }

        for (size_t j = 0; j < npt; j++ ) {
            double &leftError = imagePointsEpipolarError[0][i][j];
            double &rightError = imagePointsEpipolarError[1][i][j];

            leftError = imagePoints[0][i][j].x*lines[1][j][0] +
                    imagePoints[0][i][j].y*lines[1][j][1] +
                    lines[1][j][2];

            rightError = imagePoints[1][i][j].x*lines[0][j][0] +
                    imagePoints[1][i][j].y*lines[0][j][1] +
                    lines[0][j][2];

            double errij = fabs(leftError) + fabs(rightError);
            err += errij;
        }
        npoints += npt;
    }

    qDebug() << "average reprojection error:"
             << err/npoints
             << "pixels (sum of error is" << err << "for" << npoints << "points)";
}

ZOpenCVStereoCameraCalibration::ZOpenCVStereoCameraCalibration(const cv::Mat cameraMatrix[],
                                                               const cv::Mat distCoeffs[],
                                                               const cv::Size imageSize[],
                                                               const cv::Mat &R,
                                                               const cv::Mat &T,
                                                               const cv::Mat &E,
                                                               const cv::Mat &F)
    : ZOpenCVStereoCameraCalibration({ ZCameraCalibrationPtr(new ZPinholeCameraCalibration(cameraMatrix[0], distCoeffs[0], imageSize[0])), ZCameraCalibrationPtr(new ZPinholeCameraCalibration(cameraMatrix[1], distCoeffs[1], imageSize[1])) }, {}, {}, cameraMatrix, distCoeffs, imageSize, R, T, E, F)
{

}

} // namespace Z3D
