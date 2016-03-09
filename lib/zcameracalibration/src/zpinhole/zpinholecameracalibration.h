#pragma once

#include "zcameracalibration_global.h"
#include "zcameracalibration.h"

#include "opencv2/core/core.hpp"

#include <QPointF>

namespace Z3D
{

class Z3D_CAMERACALIBRATION_SHARED_EXPORT ZPinholeCameraCalibration : public ZCameraCalibration
{
    Q_OBJECT

    Q_PROPERTY(QPointF principalPoint READ principalPoint WRITE setPrincipalPoint NOTIFY pinholeCalibrationChanged)
    Q_PROPERTY(QPointF focalLength    READ focalLength    WRITE setFocalLength    NOTIFY pinholeCalibrationChanged)
//    Q_PROPERTY(QList<double> radialDistortion READ radialDistortion WRITE setRadialDistortion NOTIFY pinholeCalibrationChanged)
//    Q_PROPERTY(QList<double> tangentialDistortion READ tangentialDistortion WRITE setTangentialDistortion NOTIFY pinholeCalibrationChanged)

//    Q_PROPERTY(double cx READ cx WRITE setCx NOTIFY pinholeCalibrationChanged)
//    Q_PROPERTY(double cy READ cy WRITE setCy NOTIFY pinholeCalibrationChanged)

//    Q_PROPERTY(double fx READ fx WRITE setFx NOTIFY pinholeCalibrationChanged)
//    Q_PROPERTY(double fy READ fy WRITE setFy NOTIFY pinholeCalibrationChanged)

    Q_PROPERTY(double p1 READ p1 WRITE setP1 NOTIFY pinholeCalibrationChanged)
    Q_PROPERTY(double p2 READ p2 WRITE setP2 NOTIFY pinholeCalibrationChanged)

    Q_PROPERTY(double k1 READ k1 WRITE setK1 NOTIFY pinholeCalibrationChanged)
    Q_PROPERTY(double k2 READ k2 WRITE setK2 NOTIFY pinholeCalibrationChanged)
    Q_PROPERTY(double k3 READ k3 WRITE setK3 NOTIFY pinholeCalibrationChanged)
    Q_PROPERTY(double k4 READ k4 WRITE setK4 NOTIFY pinholeCalibrationChanged)
    Q_PROPERTY(double k5 READ k5 WRITE setK5 NOTIFY pinholeCalibrationChanged)
    Q_PROPERTY(double k6 READ k6 WRITE setK6 NOTIFY pinholeCalibrationChanged)

public:
    explicit ZPinholeCameraCalibration();
    explicit ZPinholeCameraCalibration(cv::Mat cameraMatrix, cv::Mat distortionCoeffs, cv::Size imageSize);

    virtual ZCameraCalibration::Ptr clone() const;

    virtual bool getCalibratedPixel(int x, int y,
                                    float *calibratedX, float *calibratedY);

    virtual bool getRayForPixel(int x, int y,           /// pixel coordinates
                                cv::Vec3d &origin,      /// ray origin
                                cv::Vec3d &direction);  /// ray direction

    virtual bool getWorldRayForPixel(int x, int y,          /// pixel coordinates
                                     cv::Vec3d &origin,     /// ray origin
                                     cv::Vec3d &direction); /// ray direction

    virtual bool getEstimatedTransformation(std::vector<cv::Point2f> &imageCorners,
                                            std::vector<cv::Point3f> &worldCorners,
                                            cv::Matx33d &rotation,
                                            cv::Point3d &translation);

    ///
    /// \brief PinholeCameraCalibration::loadCalibration
    /// Loads the calibrationi from a YML file
    /// \param fileName: read calibration from this file
    /// \return true if calibration is loaded successfully
    ///
    virtual bool loadFromFile(const QString &fileName);

    virtual bool saveToFile(const QString &fileName);

    /// Pinhole exclusive!
    cv::Mat cvCameraMatrix() { return m_cvCameraMatrix; }
    cv::Mat cvDistortionCoeffs() { return m_cvDistortionCoeffs; }

    /// camera matrix
    double cx() const;
    double cy() const;
    double fx() const;
    double fy() const;

    /// tangential distortion
    double p1() const;
    double p2() const;

    /// radial distortion
    double k1() const;
    double k2() const;
    double k3() const;
    double k4() const;
    double k5() const;
    double k6() const;

    QPointF principalPoint() const;

    QPointF focalLength() const;

//    QList<double> radialDistortion() const;

//    QList<double> tangentialDistortion() const;

signals:
    void pinholeCalibrationChanged();

public slots:
    /// camera matrix
    void setCx(double arg);
    void setCy(double arg);
    void setFx(double arg);
    void setFy(double arg);

    /// tangential distortion
    void setP1(double arg);
    void setP2(double arg);

    /// radial distortion
    void setK1(double arg);
    void setK2(double arg);
    void setK3(double arg);
    void setK4(double arg);
    void setK5(double arg);
    void setK6(double arg);

    void setPrincipalPoint(QPointF arg);

    void setFocalLength(QPointF arg);

//    void setRadialDistortion(QList<double> arg);

//    void setTangentialDistortion(QList<double> arg);

protected slots:
    void onCalibrationChanged();

protected:
    inline int indexForPixel(int x, int y) const { return x + y * m_sensorWidth; }

    ///
    /// \brief PinholeCameraCalibration::generateLookUpTable
    /// Generates the look up table to optimize the use of the calibration. The
    /// camera has discrete pixels, so we can do it without loosing anything
    ///
    void generateLookUpTable();

    cv::Mat m_cvCameraMatrix;
    cv::Mat m_cvDistortionCoeffs;

    std::vector<cv::Point2f> m_undistortedPoints;
    std::vector<cv::Vec3f> m_undistortedRays;
    std::vector<cv::Vec3f> m_undistortedWorldRays;
};

} // namespace Z3D
