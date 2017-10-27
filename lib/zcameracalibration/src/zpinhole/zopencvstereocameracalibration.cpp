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

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <QDateTime>
#include <QDebug>
#include <QDir>

namespace Z3D
{

ZOpenCVStereoCameraCalibration::ZOpenCVStereoCameraCalibration(std::vector<ZCameraCalibration::Ptr> calibrations,
                                                               std::vector<std::vector<cv::Point3f> > objectPoints,
                                                               std::vector<std::vector<std::vector<cv::Point2f> > > imagePoints,
                                                               cv::Mat cameraMatrix[2],
                                                               cv::Mat distCoeffs[2],
                                                               cv::Size imageSize[2],
                                                               cv::Mat R,
                                                               cv::Mat T,
                                                               cv::Mat E,
                                                               cv::Mat F)
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
    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    size_t nimages = imagePoints[0].size();
    for (size_t i = 0; i < nimages; i++ ) {
        const auto npt = imagePoints[0][i].size();
        cv::Mat imgpt[2];

        for (size_t k = 0; k < 2; k++ ) {
            imgpt[k] = cv::Mat(imagePoints[k][i]);
            cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
            cv::computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }

        for (size_t j = 0; j < npt; j++ ) {
            double errij =
                    fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                         imagePoints[0][i][j].y*lines[1][j][1] +
                         lines[1][j][2]) +
                    fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                         imagePoints[1][i][j].y*lines[0][j][1] +
                         lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }

    qDebug() << "average reprojection error:"
             << err/npoints
             << "pixels (sum of error is" << err << "for" << npoints << "points)";

    /// create folder where the debug images will be saved
    const QString timeStr = QDateTime::currentDateTime().toString("yyyy.MM.dd_hh.mm.ss");
    const QString calibFolder = QString("./tmp/debug/stereo/%1/").arg(timeStr);
    QDir::current().mkpath(calibFolder);
    QDir calibDir = QDir(calibFolder);

    // save intrinsic parameters
    const QString intrinsicsFileName = calibDir.absoluteFilePath("intrinsics.yml");
    cv::FileStorage fs(qPrintable(intrinsicsFileName), cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0]
           << "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    } else {
        qWarning() << "Error: cannot save the intrinsic parameters to" << intrinsicsFileName;
    }

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi[2];

    cv::Size newImageSize = imageSize[0]; // keep same size

    cv::stereoRectify(cameraMatrix[0], distCoeffs[0], // left camera calibration params
            cameraMatrix[1], distCoeffs[1], // right camera calibration params
            imageSize[0], // input image size
            R, T, R1, R2, P1, P2, Q,
            /**
             * Operation flags that may be zero or CALIB_ZERO_DISPARITY . If the flag is set,
             * the function makes the principal points of each camera have the same pixel coordinates in the
             * rectified views. And if the flag is not set, the function may still shift the images in the
             * horizontal or vertical direction (depending on the orientation of epipolar lines) to maximize the
             * useful image area.
             */
            0, // flags
            /**
             * Free scaling parameter. If it is -1 or absent, the function performs the default
             * scaling. Otherwise, the parameter should be between 0 and 1. alpha=0 means that the rectified
             * images are zoomed and shifted so that only valid pixels are visible (no black areas after
             * rectification). alpha=1 means that the rectified image is decimated and shifted so that all the
             * pixels from the original images from the cameras are retained in the rectified images (no source
             * image pixels are lost). Obviously, any intermediate value yields an intermediate result between
             * those two extreme cases.
             */
            -1, // alpha = 0 means only valid (visible from both cameras) image points are kept, and we get a validROI == newImageSize
            /**
             * New image resolution after rectification. The same size should be passed to
             * initUndistortRectifyMap (see the stereo_calib.cpp sample in OpenCV samples directory). When (0,0)
             * is passed (default), it is set to the original imageSize . Setting it to larger value can help you
             * preserve details in the original image, especially when there is a big radial distortion.
             */
            newImageSize, // keep same size
            /**
             * Optional output rectangles inside the rectified images where all the pixels
             * are valid. If alpha=0 , the ROIs cover the whole images. Otherwise, they are likely to be smaller
             */
            &validRoi[0], &validRoi[1]);

    const QString extrinsicsFileName = calibDir.absoluteFilePath("extrinsics.yml");
    fs.open(qPrintable(extrinsicsFileName), cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "R" << R << "T" << T
           << "R1" << R1 << "R2" << R2
           << "P1" << P1 << "P2" << P2
           << "Q" << Q;
        fs.release();
    } else {
        qWarning() << "Error: cannot save the extrinsic parameters to" << extrinsicsFileName;
    }

    cv::Mat rmap[2][2];
    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize[0], CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize[1], CV_16SC2, rmap[1][0], rmap[1][1]);

    const QString rectifyMapFileName = calibDir.absoluteFilePath("rmap.yml");
    fs.open(qPrintable(rectifyMapFileName), cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "M00" << rmap[0][0]
           << "M01" << rmap[0][1]
           << "M10" << rmap[1][0]
           << "M11" << rmap[1][1];
        fs.release();
    } else {
        qWarning() << "Error: cannot save the rectify map parameters to" << rectifyMapFileName;
    }
}

} // namespace Z3D
