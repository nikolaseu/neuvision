#include "zltsplanecalibration.h"

#include <Z3DCameraCalibration>

#include "opencv2/core/core.hpp" // cv::FileStorage

#include <iostream>

#include <QDebug>

namespace Z3D
{

LTSPlaneCalibration::LTSPlaneCalibration(Z3D::ZCameraCalibration::Ptr calibration) :
    m_cameraCalibration(calibration)
{
    /// (for now) use the same sensor width and height
    m_sensorWidth = m_cameraCalibration->sensorWidth();
    m_sensorHeight = m_cameraCalibration->sensorHeight();
}

void LTSPlaneCalibration::getCalibratedPointForPixel(int x, int y, float *realX, float *realY, float *realZ)
{
    /**
    - sin nada: <0.1 msec/profile
    - con calculos acá: por ahora: <2 msec/profile
                        simplificados: ~1 msec/profile
    - con lut: <0.2 msec/profile
    */

    /// get data from LUT (we always assume the LUT is already computed)
    const cv::Point3f &point = m_undistortedPoints[ indexForPixel(x, y) ];
    *realX = point.x;
    *realY = point.y;
    *realZ = point.z;
}

///
/// \brief LTSPlaneCalibration::loadCalibration
/// Loads the calibrationi from a YML file
/// \param fileName: read calibration from this file
/// \return true if calibration is loaded successfully
///
bool LTSPlaneCalibration::loadCalibration(const QString &fileName)
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

            /// load distance to plane
            fs["planeDistance"] >> m_planeDistance;

            /// load plane normal
            fs["planeNormalX"] >> m_planeNormalX;
            fs["planeNormalY"] >> m_planeNormalY;
            fs["planeNormalZ"] >> m_planeNormalZ;

            /// release file
            fs.release();

            /// show data in console
            std::cout << "Loaded LTS plane calibration from:\n " << qPrintable(fileName) << std::endl
                      << "> calibrationDate:\n " << calibrationDate << std::endl
                      << "> planeDistance:\n " << m_planeDistance << std::endl
                      << "> planeNormalX:\n " << m_planeNormalX << std::endl
                      << "> planeNormalY:\n " << m_planeNormalY << std::endl
                      << "> planeNormalZ:\n " << m_planeNormalZ << std::endl;

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

void LTSPlaneCalibration::generateLookUpTable()
{
    std::size_t pixelCount = m_sensorWidth * m_sensorHeight;

    qDebug() << "initializing LTS calibration lookup table with" << pixelCount << "values";

    /// To get the points in laser plane coordinates (z=0) we need to change to
    /// a coordinate system where the plane normal (n) is the Z axis.
    /// We only need to create the axis X and Y (the are infinite posibilities).
    /// We will denote the new coordinate system as (X_2, Y_2, Z_2).
    /// Using the camera X_c axis as starting point we get an axis Y_2 orthogonal
    /// to the plane normal (using a cross product). Then we simply create the
    /// new axis X_2 as the cross product of the plane normal and the recently
    /// obtained Y_2. In other words:
    ///     Z_2 = n
    ///     Y_2 = Z_2 x X_c
    ///     X_2 = Y_2 x Z_2
    ///
    /// Now we need to transform the camera rays to the new coordinate system,
    /// so we need the rotation and translation. The rotation is straightforward
    /// (we only use the 3 axis as column vector to form a 3x3 rotation matrix),
    /// and as translation we will use the nearest plane point wrt the camera as
    /// coordinate system origin, so the translation will be the plane normal
    /// scaled by the distance to the plane, ie:
    ///     T = d * n
    ///
    /// We need to pay special attention to the way the normal and the distance
    /// to the plane are specified. As a general rule, we will use the normal as
    /// a vector going away from the camera, and the distance is positive

    /// new coordinate system
    const cv::Vec3f newZ = cv::normalize( cv::Vec3f(m_planeNormalX, m_planeNormalY, m_planeNormalZ) );
    const cv::Vec3f newY = newZ.cross( cv::Vec3f(1.f, 0.f, 0.f) );
    const cv::Vec3f newX = newY.cross( newZ );

    cv::Mat auxRotationMatrix(3, 3, CV_32F);
    auxRotationMatrix.at<float>(0,0) = newX[0];
    auxRotationMatrix.at<float>(1,0) = newX[1];
    auxRotationMatrix.at<float>(2,0) = newX[2];
    auxRotationMatrix.at<float>(0,1) = newY[0];
    auxRotationMatrix.at<float>(1,1) = newY[1];
    auxRotationMatrix.at<float>(2,1) = newY[2];
    auxRotationMatrix.at<float>(0,2) = newZ[0];
    auxRotationMatrix.at<float>(1,2) = newZ[1];
    auxRotationMatrix.at<float>(2,2) = newZ[2];

    /// rotation matrix
    cv::Matx33f rotationMatrix(auxRotationMatrix);

    std::cout << "x:\n\t" << newX << std::endl
              << "y:\n\t" << newY << std::endl
              << "z:\n\t" << newZ << std::endl
              << "rotation:\n\t" << rotationMatrix << std::endl;

    /// translation
    const cv::Vec3f translationVector = - m_planeDistance * newZ;


    /// now the origin will be a point in the plane
    const cv::Vec3f planePoint(0, 0, 0);
    const cv::Vec3f planeNormal = newZ;
    const cv::Vec3f linePoint = translationVector;
    const cv::Vec3f planePointMinusLinePoint = planePoint - linePoint;
    const float planePointMinusLinePoint_dot_planeNormal = planePointMinusLinePoint.dot(planeNormal);
            //cv::dot<float>(planePointMinusLinePoint, planeNormal);



    //! TODO: hacer que los puntos estén en coordenadas del plano laser, asi
    //! solo tenemos coordenadas (x,y) con z=0 siempre.



    /// constants
    //const cv::Vec3f linePoint(0, 0, 0);
    //const cv::Vec3f planeNormal = cv::Vec3f(m_planeNormalX, m_planeNormalY, m_planeNormalZ);
    //const cv::Vec3f planePoint = m_planeDistance * planeNormal;
    //const cv::Vec3f planePointMinusLinePoint = planePoint - linePoint;
    //const float planePointMinusLinePoint_dot_planeNormal = cv::dot<float>(planePointMinusLinePoint, planeNormal);

    float realX,
          realY;

    /// resize
    m_undistortedPoints.resize( pixelCount );

    for (int ix = 0; ix < m_sensorWidth; ++ix) {
        for (int iy = 0; iy < m_sensorHeight; ++iy) {
            /// undistort pixel
            m_cameraCalibration->getCalibratedPixel(ix, iy, &realX, &realY);

            cv::Vec3f lineDirection = rotationMatrix * cv::normalize( cv::Vec3f(realX, realY, -1.0f) );

            /// in our case the line will never be paralell to the plane, so there shouldn't be any problem
            /// in case we want to check, dot(lineDirection,planeNormal) will be ~= zero
            float alpha = planePointMinusLinePoint_dot_planeNormal / lineDirection.dot(planeNormal); //cv::dot<float>(lineDirection, planeNormal);

            m_undistortedPoints[ indexForPixel(ix, iy) ] = cv::Point3f(
                        linePoint[0] + alpha * lineDirection[0],
                        linePoint[1] + alpha * lineDirection[1],
                        linePoint[2] + alpha * lineDirection[2]
                    );
        }
    }

    qDebug() << "finished initialization of LTS calibration lookup table";
}

} // namespace Z3D
