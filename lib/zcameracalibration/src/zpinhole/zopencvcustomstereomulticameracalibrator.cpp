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

#include "zopencvcustomstereomulticameracalibrator.h"
#include "zpinholecameracalibration.h"

#include <QDebug>

#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>


namespace Z3D
{

ZOpenCVCustomStereoMultiCameraCalibrator::ZOpenCVCustomStereoMultiCameraCalibrator(QObject *parent)
    : ZMultiCameraCalibrator(parent)
{

}

ZOpenCVCustomStereoMultiCameraCalibrator::~ZOpenCVCustomStereoMultiCameraCalibrator()
{

}

QString ZOpenCVCustomStereoMultiCameraCalibrator::name()
{
    return QLatin1String("Custom stereo calibration");
}

std::vector<ZCameraCalibration::Ptr> ZOpenCVCustomStereoMultiCameraCalibrator::getCalibration(
        std::vector< Z3D::ZCameraCalibration::Ptr > &initialCameraCalibrations,
        std::vector< std::vector< std::vector< cv::Point2f > > > &imagePoints,
        std::vector<std::vector<cv::Point3f> > &objectPoints)
{
    std::vector<ZCameraCalibration::Ptr> newCalibrations;

    const auto ncameras = initialCameraCalibrations.size();
    if (ncameras != 2) {
        qWarning() << "this only works with two cameras!";
        return newCalibrations;
    }

    std::vector<cv::Size> imageSize(ncameras);

    const auto nimages = imagePoints[0].size();
//    if (nimages < 2) {
//        qWarning() << "Error: too little pairs to run the calibration";
//        return newCalibrations;
//    }

    qDebug() << "Running stereo calibration ...";

    cv::Mat cameraMatrix[2]
          , distCoeffs[2];
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);

    for (size_t ic = 0; ic < ncameras; ++ic) {
        /// check first! this only works for pinhole cameras!
        Z3D::ZPinholeCameraCalibration *calibration = static_cast<Z3D::ZPinholeCameraCalibration*>(initialCameraCalibrations[ic].data());
        if (!calibration) {
            qWarning() << "invalid calibration! this only works for pinhole cameras!";
            return newCalibrations;
        }

        imageSize[ic] = cv::Size(calibration->sensorWidth(),
                                 calibration->sensorHeight());
        cameraMatrix[ic] = calibration->cvCameraMatrix().clone();
        distCoeffs[ic] = calibration->cvDistortionCoeffs().clone();
    }

    std::vector< std::vector<cv::Mat> > rotationMats(ncameras);
    std::vector< std::vector<cv::Mat> > translationMats(ncameras);
    std::vector< std::vector<bool> > valid(ncameras);
    for (size_t ic = 0; ic < ncameras; ++ic) {
        rotationMats[ic].resize(nimages);
        translationMats[ic].resize(nimages);
        valid[ic].resize(nimages);
    }

    /// for each camera
    for (size_t ic = 0; ic < ncameras; ++ic) {
        /// for each image pair
        for (size_t ii=0; ii<nimages; ++ii) {
            /// solve system
            valid[ic][ii] = cv::solvePnP(objectPoints[ii],
                                      imagePoints[ic][ii],
                                      cameraMatrix[ic],
                                      distCoeffs[ic],
                                      rotationMats[ic][ii],
                                      translationMats[ic][ii]);
        }
    }

    std::vector< std::vector< std::vector<double> > > allData(ncameras-1);
    for (size_t ic = 1; ic < ncameras; ++ic) {
        allData[ic-1].resize(6);
    }

    /// for each image pair
    for (size_t ii=0; ii<nimages; ++ii) {
        if (!valid[0][ii]) {
            continue;
        }

        std::vector<cv::Matx33d> rotations(ncameras-1);
        std::vector<cv::Point3d> translations(ncameras-1);

        cv::Vec3d rvec(rotationMats[0][ii]);
        cv::Matx33d firstCameraR;
        cv::Rodrigues(rvec, firstCameraR);
        cv::Point3d firstCameraT = cv::Point3d(translationMats[0][ii]);
        /// for each camera (after the first)
        size_t ic;
        for (ic = 1; ic < ncameras; ++ic) {
            if (valid[ic][ii]) {
                /// rotation was expressed as a vector, we need as matrix to apply transform easily
                /// Use rodrigues' formula
                cv::Vec3d rvec(rotationMats[ic][ii]);
                cv::Rodrigues(rvec, rotations[ic-1]);
                //rotations[ic-1] = firstCameraR.t() * rotations[ic-1]; // not ok
                rotations[ic-1] = firstCameraR * rotations[ic-1].t(); // 1: seems ok but is all inverse (need .t() at the end)
                //rotations[ic-1] = rotations[ic-1] * firstCameraR.t(); // ok2
                /// convert translation also
                //translations[ic-1] = firstCameraR * cv::Point3d(translationMats[ic][ii]) - firstCameraT; // not ok
                //translations[ic-1] = cv::Point3d(translationMats[ic][ii]) - rotations[ic-1].t() *  firstCameraT; // 1: seems ok
                //translations[ic-1] = rotations[ic-1].t() * cv::Point3d(translationMats[ic][ii]) - firstCameraT; // 2: not ok
                translations[ic-1] = firstCameraT - rotations[ic-1] * cv::Point3d(translationMats[ic][ii]); // 3: ?
            } else {
                break;
            }
        }

        if (ic==ncameras) {
            /// all camera images for this pattern position are valid
            ///
            for (ic = 1; ic < ncameras; ++ic) {
                cv::Point3d &tvec = translations[ic-1];
                /// rotation was expressed as a matrix, we'll use it as a vector to find median
                /// Use rodrigues' formula
                cv::Vec3d rvec;
                cv::Rodrigues(rotations[ic-1], rvec);
                allData[ic-1][0].push_back(rvec[0]);
                allData[ic-1][1].push_back(rvec[1]);
                allData[ic-1][2].push_back(rvec[2]);
                allData[ic-1][3].push_back(tvec.x);
                allData[ic-1][4].push_back(tvec.y);
                allData[ic-1][5].push_back(tvec.z);
            }
        }
    }

    for (size_t ic = 1; ic < ncameras; ++ic) {
        /// find median. sort and use the values in the middle of the sorted vector
        for (size_t id = 0; id < 6; ++id) {
            std::sort(allData[ic-1][id].begin(), allData[ic-1][id].end());
        }
    }

    //! FIXME debug only, show sorted vectors
    for (size_t ic = 1; ic < ncameras; ++ic) {
        for (size_t id = 0; id < 6; ++id) {
            qDebug() << QVector<double>::fromStdVector(allData[ic-1][id]);
        }
    }

    Z3D::ZCameraCalibration::Ptr cam0calib( new ZPinholeCameraCalibration(cameraMatrix[0], distCoeffs[0], imageSize[0]) );
    newCalibrations.push_back( cam0calib );

    for (size_t ic = 1; ic < ncameras; ++ic) {
        size_t size = allData[ic-1][0].size();
        if (!size) {
            qWarning() << "not enough valid calibration pattern images for camera" << ic;
            continue;
        }
        size_t middleIndex = 0;
        if (size > 2)
            middleIndex = size / 2;
        std::cout << "camera " << ic << ":" << std::endl;

        cv::Vec3d rvec;
        rvec[0] = allData[ic-1][0][middleIndex];
        rvec[1] = allData[ic-1][1][middleIndex];
        rvec[2] = allData[ic-1][2][middleIndex];

        cv::Point3d tvec;
        tvec.x = allData[ic-1][3][middleIndex];
        tvec.y = allData[ic-1][4][middleIndex];
        tvec.z = allData[ic-1][5][middleIndex];

        /// rotation was expressed as vector, convert to matrix
        /// Use rodrigues' formula
        cv::Matx33d rMat;
        cv::Rodrigues(rvec, rMat);

        std::cout << "rotation:" << std::endl << rMat << std::endl;
        std::cout << "translation:" << std::endl << tvec << std::endl;

        std::cout << "rotation_t:" << std::endl << rMat.t() << std::endl;
        std::cout << "translation_rMat*trans:" << std::endl << rMat * tvec << std::endl;
        std::cout << "translation_rMat_t*trans:" << std::endl << rMat.t() * tvec << std::endl;

        Z3D::ZCameraCalibration::Ptr camCalib( new ZPinholeCameraCalibration(cameraMatrix[ic], distCoeffs[ic], imageSize[ic]) );
        newCalibrations.push_back( camCalib );

        //! this is wrong but this is the way the software works now
        //! FIXME fix everything!
        cam0calib->setRotation(rMat.t());
        cam0calib->setTranslation(-(rMat.t() * tvec));
    }

    return newCalibrations;
}

bool ZOpenCVCustomStereoMultiCameraCalibrator::fixIntrinsic() const
{
    return m_fixIntrinsic;
}

void ZOpenCVCustomStereoMultiCameraCalibrator::setFixIntrinsic(bool arg)
{
    if (m_fixIntrinsic == arg)
        return;

    m_fixIntrinsic = arg;
    emit fixIntrinsicChanged(arg);
}

} // namespace Z3D
