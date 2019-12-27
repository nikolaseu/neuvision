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

#include "ZCameraCalibration/zopencvstereocameracalibration.h"

#include "ZCameraCalibration/zpinholecameracalibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <QDebug>

namespace Z3D
{

ZOpenCVStereoCameraCalibration::ZOpenCVStereoCameraCalibration(const std::vector<ZCameraCalibrationPtr> &calibrations,
                                                               const std::vector<std::vector<cv::Point3f> > &objectPoints_,
                                                               const std::vector<std::vector<std::vector<cv::Point2f> > > &imagePoints_,
                                                               const cv::Mat cameraMatrix_[],
                                                               const cv::Mat distCoeffs_[],
                                                               const cv::Size imageSize_[],
                                                               const cv::Mat &R_,
                                                               const cv::Mat &T_,
                                                               const cv::Mat &E_,
                                                               const cv::Mat &F_)
    : ZMultiCameraCalibration(calibrations)
    , objectPoints(objectPoints_)
    , imagePoints(imagePoints_)
    , cameraMatrix { cameraMatrix_[0], cameraMatrix_[1] }
    , distCoeffs { distCoeffs_[0], distCoeffs_[1] }
    , imageSize { imageSize_[0], imageSize_[1] }
    , R(R_)
    , T(T_)
    , E(E_)
    , F(F_)
{
    const auto ncameras = 2;
    const auto nimages = imagePoints.empty() ? 0 : imagePoints[0].size();
    imagePointsEpipolarError.resize(ncameras);
    for (size_t cam=0; cam<ncameras; ++cam) {
        imagePointsEpipolarError[cam].resize(nimages);
        for (size_t image=0; image<nimages; ++image) {
            const size_t npoints = imagePoints[cam][image].size();
            imagePointsEpipolarError[cam][image].resize(npoints);
        }
    }

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    size_t npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for (size_t i = 0; i < nimages; i++ ) {
        const auto npt = imagePoints[0][i].size();
        cv::Mat imgpt[2];

        for (size_t k = 0; k < 2; k++ ) {
            imgpt[k] = cv::Mat(imagePoints[k][i]);
            cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
            cv::computeCorrespondEpilines(imgpt[k], int(k+1), F, lines[k]);
        }

        for (size_t j = 0; j < npt; j++ ) {
            float &leftError = imagePointsEpipolarError[0][i][j];
            float &rightError = imagePointsEpipolarError[1][i][j];

            leftError = imagePoints[0][i][j].x*lines[1][j][0] +
                    imagePoints[0][i][j].y*lines[1][j][1] +
                    lines[1][j][2];

            rightError = imagePoints[1][i][j].x*lines[0][j][0] +
                    imagePoints[1][i][j].y*lines[0][j][1] +
                    lines[0][j][2];

            const double errij = double(fabs(leftError) + fabs(rightError));
            err += errij;
        }
        npoints += npt;
    }

    qDebug() << "average reprojection error:"
             << err/npoints
             << "pixels (sum of error is" << err << "for" << npoints << "points)";
}

ZOpenCVStereoCameraCalibration::ZOpenCVStereoCameraCalibration(const cv::Mat cameraMatrix_[],
                                                               const cv::Mat distCoeffs_[],
                                                               const cv::Size imageSize_[],
                                                               const cv::Mat &R_,
                                                               const cv::Mat &T_,
                                                               const cv::Mat &E_,
                                                               const cv::Mat &F_)
    : ZOpenCVStereoCameraCalibration({ std::make_shared<ZPinholeCameraCalibration>(cameraMatrix_[0], distCoeffs_[0], imageSize_[0]),
                                       std::make_shared<ZPinholeCameraCalibration>(cameraMatrix_[1], distCoeffs_[1], imageSize_[1]) },
                                     {}, {}, cameraMatrix_, distCoeffs_, imageSize_, R_, T_, E_, F_)
{

}

} // namespace Z3D
