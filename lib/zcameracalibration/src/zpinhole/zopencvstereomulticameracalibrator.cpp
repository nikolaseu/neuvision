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

#include <QDateTime>
#include <QDebug>
#include <QDir>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace Z3D
{

ZOpenCVStereoMultiCameraCalibrator::ZOpenCVStereoMultiCameraCalibrator(QObject *parent)
    : ZMultiCameraCalibrator(parent)
    , m_fixIntrinsic(false)
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
    , m_isDebugMode(true)
{

}

ZOpenCVStereoMultiCameraCalibrator::~ZOpenCVStereoMultiCameraCalibrator()
{

}

QString ZOpenCVStereoMultiCameraCalibrator::id() const
{
    return QString(metaObject()->className());
}

QString ZOpenCVStereoMultiCameraCalibrator::name() const
{
    return QLatin1String("OpenCV stereo calibration");
}

std::vector<ZCameraCalibration::Ptr> ZOpenCVStereoMultiCameraCalibrator::getCalibration(
        std::vector< Z3D::ZCameraCalibration::Ptr > &initialCameraCalibrations,
        std::vector< std::vector< std::vector< cv::Point2f > > > &imagePoints,
        std::vector< std::vector< cv::Point3f > > &objectPoints) const
{
    std::vector<ZCameraCalibration::Ptr> newCalibrations;

    size_t ncameras = initialCameraCalibrations.size();
    if (ncameras != 2) {
        qWarning() << "this only works with two cameras!";
        return newCalibrations;
    }

    std::vector<cv::Size> imageSize(ncameras);

    size_t nimages = imagePoints[0].size();
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
          , F;

    for (size_t ic = 0; ic < ncameras; ++ic) {
        /// check first! this only works for pinhole cameras!
        Z3D::ZPinholeCameraCalibration *calibration = static_cast<Z3D::ZPinholeCameraCalibration*>(initialCameraCalibrations[ic].data());
        if (!calibration) {
            qWarning() << "invalid calibration! this only works for pinhole cameras!";
            return newCalibrations;
        }

        if (calibration->sensorWidth() * calibration->sensorWidth() < 1) {
            qWarning() << "invalid calibration config! camera sensor size is zero!";
            return newCalibrations;
        }

        imageSize[ic] = cv::Size(calibration->sensorWidth(),
                                 calibration->sensorHeight());
        cameraMatrix[ic] = calibration->cvCameraMatrix().clone();
        distCoeffs[ic] = calibration->cvDistortionCoeffs().clone();

        qDebug() << "camera" << ic << "imageSize:" << imageSize[ic].width << "x" << imageSize[ic].height;
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
            objectPoints,
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

    qDebug() << "stereo calibration finished with RMS error:" << rms
             << ", distance between cameras:" << cv::norm(T);

    if (m_isDebugMode) {

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

        cv::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
                imageSize[0], R, T, R1, R2, P1, P2, Q, 0, -1, imageSize[0], &validRoi[0], &validRoi[1]);

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

    for (size_t ic = 0; ic < ncameras; ++ic) {
        Z3D::ZCameraCalibration::Ptr calibration(
                    new Z3D::ZPinholeCameraCalibration(cameraMatrix[ic], distCoeffs[ic], imageSize[ic]));
        if (ic == 0) {
            calibration->setRotation(R);
            calibration->setTranslation(T);
        }
        newCalibrations.push_back(calibration);
    }

    return newCalibrations;
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

bool ZOpenCVStereoMultiCameraCalibrator::isDebugMode() const
{
    return m_isDebugMode;
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
    if (fabs(m_termCriteriaEpsilon - arg) > DBL_EPSILON) {
        m_termCriteriaEpsilon = arg;
        emit termCriteriaEpsilonChanged(arg);
    }
}

void ZOpenCVStereoMultiCameraCalibrator::setDebugMode(bool isDebugMode)
{
    if (m_isDebugMode == isDebugMode)
        return;

    m_isDebugMode = isDebugMode;
    emit debugModeChanged(m_isDebugMode);
}

} // namespace Z3D
