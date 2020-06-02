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

#include "ZCameraCalibration/zpinholecameracalibrator.h"

#include "ZCameraCalibration/zpinholecameracalibration.h"
#include "ZCameraCalibration/zpinholecameracalibratorconfigwidget.h"

#include "ZCore/zlogging.h"

#include <opencv2/calib3d.hpp>

#include <iostream>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameracalibration", QtInfoMsg)

namespace Z3D
{

ZPinholeCameraCalibrator::ZPinholeCameraCalibrator(QObject *parent)
    : ZCameraCalibrator(parent)
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
{

}

ZPinholeCameraCalibrator::~ZPinholeCameraCalibrator()
{

}

QString ZPinholeCameraCalibrator::id() const
{
    return QString(metaObject()->className());
}

QString ZPinholeCameraCalibrator::name() const
{
    return QLatin1String("Pinhole + distorsion (radial & tangential)");
}

ZCameraCalibrationPtr ZPinholeCameraCalibrator::getCalibration(
        std::vector<std::vector<cv::Point2f> > &imagePoints,
        std::vector<std::vector<cv::Point3f> > &realWorldPoints,
        cv::Size &imageSize)
{
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;

    /// load calibration flags
    int calibrationFlags = 0;
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

    {
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;

        /// start calibration method
        double errorRms = cv::calibrateCamera(realWorldPoints, imagePoints,
                                              imageSize,
                                              cameraMatrix, distortionCoeffs,
                                              rvecs, tvecs,
                                              calibrationFlags,
                                              cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                                               m_termCriteriaMaxIterations,
                                                               m_termCriteriaEpsilon));

        /// process results
        std::cout << cameraMatrix << std::endl;
        std::cout << distortionCoeffs << std::endl;

        zDebug() << "Error:" << errorRms;
    }

    return ZCameraCalibrationPtr(new Z3D::ZPinholeCameraCalibration(cameraMatrix, distortionCoeffs, imageSize));
}

bool ZPinholeCameraCalibrator::useIntrinsicGuess() const
{
    return m_useIntrinsicGuess;
}

bool ZPinholeCameraCalibrator::fixPrincipalPoint() const
{
    return m_fixPrincipalPoint;
}

bool ZPinholeCameraCalibrator::fixAspectRatio() const
{
    return m_fixAspectRatio;
}

bool ZPinholeCameraCalibrator::zeroTangentialDistortion() const
{
    return m_zeroTangentialDistortion;
}

bool ZPinholeCameraCalibrator::fixK1() const
{
    return m_fixK1;
}

bool ZPinholeCameraCalibrator::fixK2() const
{
    return m_fixK2;
}

bool ZPinholeCameraCalibrator::fixK3() const
{
    return m_fixK3;
}

bool ZPinholeCameraCalibrator::fixK4() const
{
    return m_fixK4;
}

bool ZPinholeCameraCalibrator::fixK5() const
{
    return m_fixK5;
}

bool ZPinholeCameraCalibrator::fixK6() const
{
    return m_fixK6;
}

bool ZPinholeCameraCalibrator::useRationalModel() const
{
    return m_useRationalModel;
}

int ZPinholeCameraCalibrator::termCriteriaMaxIterations() const
{
    return m_termCriteriaMaxIterations;
}

double ZPinholeCameraCalibrator::termCriteriaEpsilon() const
{
    return m_termCriteriaEpsilon;
}

void ZPinholeCameraCalibrator::setUseIntrinsicGuess(bool arg)
{
    if (m_useIntrinsicGuess != arg) {
        m_useIntrinsicGuess = arg;
        emit useIntrinsicGuessChanged(arg);
    }
}

void ZPinholeCameraCalibrator::setFixPrincipalPoint(bool arg)
{
    if (m_fixPrincipalPoint != arg) {
        m_fixPrincipalPoint = arg;
        emit fixPrincipalPointChanged(arg);
    }
}

void ZPinholeCameraCalibrator::setFixAspectRatio(bool arg)
{
    if (m_fixAspectRatio != arg) {
        m_fixAspectRatio = arg;
        emit fixAspectRatioChanged(arg);
    }
}

void ZPinholeCameraCalibrator::setZeroTangentialDistortion(bool arg)
{
    if (m_zeroTangentialDistortion != arg) {
        m_zeroTangentialDistortion = arg;
        emit zeroTangentialDistortionChanged(arg);
    }
}

void ZPinholeCameraCalibrator::setFixK1(bool arg)
{
    if (m_fixK1 != arg) {
        m_fixK1 = arg;
        emit fixK1Changed(arg);
    }
}

void ZPinholeCameraCalibrator::setFixK2(bool arg)
{
    if (m_fixK2 != arg) {
        m_fixK2 = arg;
        emit fixK2Changed(arg);
    }
}

void ZPinholeCameraCalibrator::setFixK3(bool arg)
{
    if (m_fixK3 != arg) {
        m_fixK3 = arg;
        emit fixK3Changed(arg);
    }
}

void ZPinholeCameraCalibrator::setFixK4(bool arg)
{
    if (m_fixK4 != arg) {
        m_fixK4 = arg;
        emit fixK4Changed(arg);
    }
}

void ZPinholeCameraCalibrator::setFixK5(bool arg)
{
    if (m_fixK5 != arg) {
        m_fixK5 = arg;
        emit fixK5Changed(arg);
    }
}

void ZPinholeCameraCalibrator::setFixK6(bool arg)
{
    if (m_fixK6 != arg) {
        m_fixK6 = arg;
        emit fixK6Changed(arg);
    }
}

void ZPinholeCameraCalibrator::setUseRationalModel(bool arg)
{
    if (m_useRationalModel != arg) {
        m_useRationalModel = arg;
        emit useRationalModelChanged(arg);
    }
}

void ZPinholeCameraCalibrator::setTermCriteriaMaxIterations(int arg)
{
    if (m_termCriteriaMaxIterations != arg) {
        m_termCriteriaMaxIterations = arg;
        emit termCriteriaMaxIterationsChanged(arg);
    }
}

void ZPinholeCameraCalibrator::setTermCriteriaEpsilon(double arg)
{
    if (m_termCriteriaEpsilon != arg) {
        m_termCriteriaEpsilon = arg;
        emit termCriteriaEpsilonChanged(arg);
    }
}

} // namespace Z3D
