#include "pinholecameracalibration.h"

#include <opencv2/imgproc/imgproc.hpp> // undistortPoints

#include <iostream>

#include <QDebug>

namespace Z3D
{

PinholeCameraCalibration::PinholeCameraCalibration() :
    CameraCalibrationInterface()
{
}

///
/// \brief PinholeCameraCalibration::getCalibratedPixel
/// Returns the calibrated position for the pixel at (x,y)
/// \param x: "x" coordinate of the pixel
/// \param y: "y" coordinate of the pixel
/// \param calibratedX: calibrated "x" coordinate for the pixel
/// \param calibratedY: calibrated "y" coordinate for the pixel
///
void PinholeCameraCalibration::getCalibratedPixel(int x, int y, float *calibratedX, float *calibratedY)
{
    const cv::Point2f &undistortedPoint = m_undistortedPoints[ indexForPixel(x, y) ];
    *calibratedX = undistortedPoint.x;
    *calibratedY = undistortedPoint.y;
}

void PinholeCameraCalibration::getRayForPixel(int x, int y, float *origin, float *direction)
{
    origin[0] = m_cvTranslation[0];
    origin[1] = m_cvTranslation[1];
    origin[2] = m_cvTranslation[2];

    const cv::Point2f &undistortedPoint = m_undistortedPoints[ indexForPixel(x, y) ];
    direction[0] = undistortedPoint.x;
    direction[1] = undistortedPoint.y;
    direction[2] = 1.f;
}

///
/// \brief PinholeCameraCalibration::loadCalibration
/// Loads the calibrationi from a YML file
/// \param fileName: read calibration from this file
/// \return true if calibration is loaded successfully
///
bool PinholeCameraCalibration::loadCalibration(const QString &fileName)
{
    if (fileName.isEmpty() || fileName.isNull())
        return false;

    /// catch exception thrown by OpenCV when something goes wrong with the file
    /// or format, we don't want to crash the program
    try {
        /// open file
        cv::FileStorage fs(qPrintable(fileName), cv::FileStorage::READ);
        if( fs.isOpened() ) {
            /// load calibration date
            std::string calibrationDate;
            fs["calibrationDate"] >> calibrationDate;

            /// load sensor size
            fs["sensorWidth"] >> m_sensorWidth;
            fs["sensorHeight"] >> m_sensorHeight;

            /// load intrinsic parameters
            cv::Mat cameraMatrix;
            fs["cameraMatrix"] >> cameraMatrix;
            if (cameraMatrix.empty()) {
                qWarning() << "Error: unable to load camera matrix";
                return false;
            }

            cv::Mat distCoeffs;
            fs["distCoeffs"] >> distCoeffs;
            if (distCoeffs.empty()) {
                qWarning() << "Error: unable to load distortion coeffs";
                return false;
            }

            /// load extrinsic parameters (optional)
            cv::Mat rotation;
            fs["rotation"] >> rotation;
            if (rotation.empty()) {
                qDebug() << "skipping rotation...";
                rotation = cv::Mat::eye(3, 3, CV_64F);
            }

            cv::Mat translation;
            fs["translation"] >> translation;
            if (translation.empty()) {
                qDebug() << "skipping translation...";
                translation = cv::Mat::zeros(3, 1, CV_64F);
            }

            /// release file
            fs.release();

            /// convert to double precision and load into class variables
            cameraMatrix.convertTo(m_cvCameraMatrix, CV_64F);
            distCoeffs.convertTo(m_cvDistortionCoeffs, CV_64F);
            rotation.convertTo(m_cvRotation, CV_64F);
            translation.convertTo(m_cvTranslation, CV_64F);

            /// show data in console
            std::cout << "Loaded pinhole camera calibration from:\n " << qPrintable(fileName) << std::endl
                      << "> calibrationDate:\n " << calibrationDate << std::endl
                      << "> sensorWidth:\n " << m_sensorWidth << std::endl
                      << "> sensorHeight:\n " << m_sensorHeight << std::endl
                      << "> cameraMatrix:\n " << m_cvCameraMatrix << std::endl
                      << "> distCoeffs:\n " << m_cvDistortionCoeffs << std::endl
                      << "> rotation:\n " << m_cvRotation << std::endl
                      << "> translation:\n " << m_cvTranslation << std::endl;

            /// copy to CameraCalibrationInterfase data
            for (int i=0; i<3; ++i) {
                for (int j=0; j<3; ++j) {
                    m_rotation[i*3+j] = rotation.at<double>(i,j);
                }
            }
            qDebug() << m_rotation[0] << m_rotation[1] << m_rotation[2]
                     << m_rotation[3] << m_rotation[4] << m_rotation[5]
                     << m_rotation[6] << m_rotation[7] << m_rotation[8];

            for (int i=0; i<3; ++i)
                m_translation[i] = m_cvTranslation[i];

            qDebug() << m_translation[0] << m_translation[1] << m_translation[2];

            /// generate calibration LUT
            generateLookUpTable();

            return true;
        }
    } catch (...) {
        /// something went wrong, but we don't care.
        /// it's just that we don't want the program to crash.
        /// the user should check if this function return true or false!
    }

    qWarning() << "Error: can not open the calibration file" << fileName;
    return false;
}

///
/// \brief PinholeCameraCalibration::generateLookUpTable
/// Generates the look up table to optimize the use of the calibration. The
/// camera has discrete pixels, so we can do it without loosing anything
///
void PinholeCameraCalibration::generateLookUpTable()
{
    std::size_t pixelCount = m_sensorWidth * m_sensorHeight;

    qDebug() << "initializing undistorted rays lookup table with" << pixelCount << "values";

    /// fill "distorted" points for every pixel in the image
    std::vector<cv::Point2f> points;
    points.resize( pixelCount );
    for (int ix = 0; ix < m_sensorWidth; ++ix) {
        for (int iy = 0; iy < m_sensorHeight; ++iy) {
            points[ indexForPixel(ix, iy) ] = cv::Point2f(ix, iy);
        }
    }

    /// undistorted points will be returned here
    m_undistortedPoints.resize( pixelCount );

    /// undistortedPoints will already be in object space, not pixels, since we don't specify "P" matrix below
    cv::undistortPoints(points, m_undistortedPoints, m_cvCameraMatrix, m_cvDistortionCoeffs /*, InputArray R=noArray(), InputArray P=noArray()*/);

/*
 * UNUSED FOR NOW ....
 *
    m_undistortedRays.resize( pixelCount );
    m_undistortedWorldRays.resize( pixelCount );

    cv::Vec3f ray;

    for (int ix = 0; ix < mSensorWidth; ++ix) {
        for (int iy = 0; iy < mSensorHeight; ++iy) {
            int index = indexForPixel(ix, iy);

            const cv::Point2f &undistortedPoint = mUndistortedPoints[index];

            ray[0] = undistortedPoint.x;
            ray[1] = undistortedPoint.y;
            ray[2] = 1.;

            /// store undistorted ray
            m_undistortedRays[index] = ray;

            /// store "world" ray
            /// use normalized (unit vector) to optimize intersection calculation
            m_undistortedWorldRays[index] = m_rotation * cv::normalize(ray);
        }
    }
*/
    qDebug() << "finished initialization of undistorted rays lookup table";
}

} // namespace Z3D
