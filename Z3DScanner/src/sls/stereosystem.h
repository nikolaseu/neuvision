#ifndef STEREOSYSTEM_H
#define STEREOSYSTEM_H

#include <Z3DCalibratedCamera>
#include <Z3DPointCloud>

#include "zpinhole/zpinholecameracalibration.h"

#include <QObject>
#include <QMap>

#include <opencv2/core/core.hpp>

struct ParallelFringeProcessingImpl;

class StereoSystem : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool ready READ ready WRITE setReady NOTIFY readyChanged)

public:
    friend struct ParallelFringeProcessingImpl;

    explicit StereoSystem(QObject *parent = 0);
    ~StereoSystem();

    Z3D::ZCalibratedCamera::Ptr camera1();
    Z3D::ZCalibratedCamera::Ptr camera2();

    void setCamera1(Z3D::ZCalibratedCamera::Ptr camera);
    void setCamera2(Z3D::ZCalibratedCamera::Ptr camera);

    bool ready() const;

signals:
    void readyChanged(bool arg);

public slots:
    void stereoRectify(double alpha = -1);

//    cv::Mat getRectifiedSnapshot2D();
    PointCloudPCLPtr getRectifiedSnapshot3D(int cameraIndex, float imageScale = 50, int lod_step = 4);

    PointCloudPCLPtr triangulateOptimized(const cv::Mat &intensityImg,
                                          std::map<int, std::vector<cv::Vec2f> > &leftPoints,
                                          std::map<int, std::vector<cv::Vec2f> > &rightPoints,
                                          int maxPosibleCloudPoints,
                                          float maxValidDistanceThreshold);

protected slots:
    void setCamera1Calibration(Z3D::ZCameraCalibration::Ptr cameraCalibration);
    void setCamera2Calibration(Z3D::ZCameraCalibration::Ptr cameraCalibration);

    void precomputeOptimizations();

    inline int indexForPixel(int x, int y) const { return x + y * m_imageSize.width; }

    void setReady(bool arg);

protected:
    std::vector<Z3D::ZCalibratedCamera::Ptr> mCam;
    std::vector<QPointer<Z3D::ZPinholeCameraCalibration> > mCal;

    std::vector< std::vector<cv::Vec3d> > m_undistortedRays;
    //std::vector< std::vector<cv::Vec3d> > m_undistortedWorldRays;

    cv::Size m_imageSize;

    std::vector<cv::Mat> m_R;
    std::vector<cv::Mat> m_P;
    cv::Mat m_Q;

private:
    bool m_ready;
};

#endif // STEREOSYSTEM_H
