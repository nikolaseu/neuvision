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

#include "zopencvstereomulticameracalibrator.h"

#include "zpinholecameracalibration.h"
#include "zopencvstereomulticameracalibratorconfigwidget.h"

#include <QDebug>
#include <QGroupBox>
#include <QLabel>
#include <QVBoxLayout>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

namespace Z3D
{

ZOpenCVStereoMultiCameraCalibrator::ZOpenCVStereoMultiCameraCalibrator(QObject *parent)
    : ZMultiCameraCalibrator(parent)
    , m_fixIntrinsic(true)
    , m_sameFocalLength(false)
    , m_useIntrinsicGuess(false)
    , m_fixPrincipalPoint(false)
    , m_fixAspectRatio(false)
    , m_zeroTangentialDistortion(false)
    , m_fixK1(false)
    , m_fixK2(false)
    , m_fixK3(false)
    , m_fixK4(false)
    , m_fixK5(false)
    , m_fixK6(false)
    , m_useRationalModel(false)
    , m_termCriteriaMaxIterations(50)
    , m_termCriteriaEpsilon(1e-7)
    , m_configWidget(nullptr)
{

}

ZOpenCVStereoMultiCameraCalibrator::~ZOpenCVStereoMultiCameraCalibrator()
{
    if (m_configWidget)
        m_configWidget->deleteLater();
}

QString ZOpenCVStereoMultiCameraCalibrator::name()
{
    return QLatin1String("OpenCV stereo calibration");
}

std::vector<ZCameraCalibration::Ptr> ZOpenCVStereoMultiCameraCalibrator::getCalibration(
        std::vector< Z3D::ZCameraCalibration::Ptr > &initialCameraCalibrations,
        std::vector< std::vector< std::vector< cv::Point2f > > > &imagePoints,
        std::vector< std::vector< std::vector< cv::Point3f > > > &objectPoints)
{
    qDebug() << "starting" << Q_FUNC_INFO;

    std::vector<ZCameraCalibration::Ptr> newCalibrations;

    int ncameras = initialCameraCalibrations.size();
    if (ncameras != 2) {
        qWarning() << "this only works with two cameras!";
        return newCalibrations;
    }

    std::vector<cv::Size> imageSize(ncameras);

    int nimages = imagePoints[0].size();
//    if (nimages < 2) {
//        qWarning() << "Error: too little pairs to run the calibration";
//        return newCalibrations;
//    }

    qDebug() << "Running stereo calibration ...";

    cv::Mat cameraMatrix[2]
          , distCoeffs[2]
          , R
          , T
          , E
          , F;;
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);

    for (int ic = 0; ic < ncameras; ++ic) {
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

    /// load calibration flags
    int calibrationFlags = 0;
    if (m_fixIntrinsic) {
        calibrationFlags = cv::CALIB_FIX_INTRINSIC;
    } else {
        if (m_sameFocalLength)
            calibrationFlags |= cv::CALIB_SAME_FOCAL_LENGTH;
        /*
        cv::CALIB_FIX_FOCAL_LENGTH
                cv::CALIB_FIX_S1_S2_S3_S4
                cv::CALIB_THIN_PRISM_MODEL
                cv::CALIB_ZERO_DISPARITY
                cv::CALIB_USE_INTRINSIC_GUESS
        */
        if (m_useIntrinsicGuess)
            calibrationFlags |= cv::CALIB_USE_INTRINSIC_GUESS;
        if (m_fixPrincipalPoint)
            calibrationFlags |= cv::CALIB_FIX_PRINCIPAL_POINT;
        if (m_fixAspectRatio)
            calibrationFlags |= cv::CALIB_FIX_ASPECT_RATIO;
        if (m_zeroTangentialDistortion)
            calibrationFlags |= cv::CALIB_ZERO_TANGENT_DIST;
        if (m_fixK1)
            calibrationFlags |= cv::CALIB_FIX_K1;
        if (m_fixK2)
            calibrationFlags |= cv::CALIB_FIX_K2;
        if (m_fixK3)
            calibrationFlags |= cv::CALIB_FIX_K3;
        if (m_fixK4)
            calibrationFlags |= cv::CALIB_FIX_K4;
        if (m_fixK5)
            calibrationFlags |= cv::CALIB_FIX_K5;
        if (m_fixK6)
            calibrationFlags |= cv::CALIB_FIX_K6;
        if (m_useRationalModel)
            calibrationFlags |= cv::CALIB_RATIONAL_MODEL;
    }

    double rms = cv::stereoCalibrate(
            objectPoints[0],
            imagePoints[0], imagePoints[1],
            cameraMatrix[0], distCoeffs[0],
            cameraMatrix[1], distCoeffs[1],
            imageSize[0], R, T, E, F,
        #ifdef CV_VERSION_EPOCH
            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, m_termCriteriaMaxIterations, m_termCriteriaEpsilon),
            calibrationFlags
        #else
            calibrationFlags,
            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, m_termCriteriaMaxIterations, m_termCriteriaEpsilon)
        #endif
            );

    qDebug() << "\t-> done with RMS error:" << rms;

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for (int i = 0; i < nimages; i++ ) {
        int npt = (int)imagePoints[0][i].size();
        cv::Mat imgpt[2];

        for (int k = 0; k < 2; k++ ) {
            imgpt[k] = cv::Mat(imagePoints[k][i]);
            cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
            cv::computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }

        for (int j = 0; j < npt; j++ ) {
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

    qDebug() << "average reprojection err = " << err/npoints;

/*
    // save each camera intrinsic and extrinsic parameters
    QString timeStr = QDateTime::currentDateTime().toString("yyyy.MM.dd_hh.mm.ss");
    for( k = 0; k < 2; k++ ) {
        cv::FileStorage fs(qPrintable(QString("%1/%2_%3.calib.yml").arg(dir).arg(timeStr).arg(camerasFolders[k])), CV_STORAGE_WRITE);
        if( fs.isOpened() ) {
            fs << "calibrationDate" << qPrintable(QDateTime::currentDateTime().toString(Qt::ISODate));
            fs << "sensorWidth" << imageSize[k].width;
            fs << "sensorHeight" << imageSize[k].height;
            fs << "cameraMatrix" << cameraMatrix[k] << "distCoeffs" << distCoeffs[k];
            if (k == 0) {
                fs << "rotation" << R << "translation" << T;
            } else {
                fs << "rotation" << cv::Mat::eye(3, 3, CV_64F) << "translation" << cv::Mat::zeros(3, 1, CV_64F);
            }
            fs.release();
        } else {
            qCritical() << "Error: can not save the calibration parameters for camera" << camerasFolders[k];
        }
    }
*/

    for (int ic = 0; ic < ncameras; ++ic) {
        Z3D::ZCameraCalibration::Ptr calibration(
                    new Z3D::ZPinholeCameraCalibration(cameraMatrix[ic], distCoeffs[ic], imageSize[ic]));
        if (ic == 0) {
            std::cout << "rotation:" << std::endl << R << std::endl;
            std::cout << "translation:" << std::endl << T << std::endl;

            calibration->setRotation(R);
            calibration->setTranslation(T);
        }
        newCalibrations.push_back(calibration);
    }

    return newCalibrations;
}

QWidget *ZOpenCVStereoMultiCameraCalibrator::configWidget()
{
    if (!m_configWidget)
        m_configWidget = new ZOpenCVStereoMultiCameraCalibratorConfigWidget(this);

    return m_configWidget;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixIntrinsic() const
{
    return m_fixIntrinsic;
}

bool ZOpenCVStereoMultiCameraCalibrator::sameFocalLength() const
{
    return m_sameFocalLength;
}

bool ZOpenCVStereoMultiCameraCalibrator::useIntrinsicGuess() const
{
    return m_useIntrinsicGuess;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixPrincipalPoint() const
{
    return m_fixPrincipalPoint;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixAspectRatio() const
{
    return m_fixAspectRatio;
}

bool ZOpenCVStereoMultiCameraCalibrator::zeroTangentialDistortion() const
{
    return m_zeroTangentialDistortion;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixK1() const
{
    return m_fixK1;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixK2() const
{
    return m_fixK2;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixK3() const
{
    return m_fixK3;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixK4() const
{
    return m_fixK4;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixK5() const
{
    return m_fixK5;
}

bool ZOpenCVStereoMultiCameraCalibrator::fixK6() const
{
    return m_fixK6;
}

bool ZOpenCVStereoMultiCameraCalibrator::useRationalModel() const
{
    return m_useRationalModel;
}

int ZOpenCVStereoMultiCameraCalibrator::termCriteriaMaxIterations() const
{
    return m_termCriteriaMaxIterations;
}

double ZOpenCVStereoMultiCameraCalibrator::termCriteriaEpsilon() const
{
    return m_termCriteriaEpsilon;
}

void ZOpenCVStereoMultiCameraCalibrator::setFixIntrinsic(bool arg)
{
    if (m_fixIntrinsic == arg)
        return;

    m_fixIntrinsic = arg;
    emit fixIntrinsicChanged(arg);
}

void ZOpenCVStereoMultiCameraCalibrator::setSameFocalLength(bool arg)
{
    if (m_sameFocalLength == arg)
        return;

    m_sameFocalLength = arg;
    emit sameFocalLengthChanged(arg);
}

void ZOpenCVStereoMultiCameraCalibrator::setUseIntrinsicGuess(bool arg)
{
    if (m_useIntrinsicGuess != arg) {
        m_useIntrinsicGuess = arg;
        emit useIntrinsicGuessChanged(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setFixPrincipalPoint(bool arg)
{
    if (m_fixPrincipalPoint != arg) {
        m_fixPrincipalPoint = arg;
        emit fixPrincipalPointChanged(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setFixAspectRatio(bool arg)
{
    if (m_fixAspectRatio != arg) {
        m_fixAspectRatio = arg;
        emit fixAspectRatioChanged(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setZeroTangentialDistortion(bool arg)
{
    if (m_zeroTangentialDistortion != arg) {
        m_zeroTangentialDistortion = arg;
        emit zeroTangentialDistortionChanged(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setFixK1(bool arg)
{
    if (m_fixK1 != arg) {
        m_fixK1 = arg;
        emit fixK1Changed(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setFixK2(bool arg)
{
    if (m_fixK2 != arg) {
        m_fixK2 = arg;
        emit fixK2Changed(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setFixK3(bool arg)
{
    if (m_fixK3 != arg) {
        m_fixK3 = arg;
        emit fixK3Changed(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setFixK4(bool arg)
{
    if (m_fixK4 != arg) {
        m_fixK4 = arg;
        emit fixK4Changed(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setFixK5(bool arg)
{
    if (m_fixK5 != arg) {
        m_fixK5 = arg;
        emit fixK5Changed(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setFixK6(bool arg)
{
    if (m_fixK6 != arg) {
        m_fixK6 = arg;
        emit fixK6Changed(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setUseRationalModel(bool arg)
{
    if (m_useRationalModel != arg) {
        m_useRationalModel = arg;
        emit useRationalModelChanged(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setTermCriteriaMaxIterations(int arg)
{
    if (m_termCriteriaMaxIterations != arg) {
        m_termCriteriaMaxIterations = arg;
        emit termCriteriaMaxIterationsChanged(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setTermCriteriaEpsilon(double arg)
{
    if (m_termCriteriaEpsilon != arg) {
        m_termCriteriaEpsilon = arg;
        emit termCriteriaEpsilonChanged(arg);
    }
}

} // namespace Z3D
