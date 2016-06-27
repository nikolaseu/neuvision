#pragma once

#include "zcalibratedcamera.h"
#include "zsimplepointcloud.h"

#include "zpinhole/zpinholecameracalibration.h"

#include <QObject>

#include "opencv2/core/types.hpp"

namespace Z3D
{

struct ParallelFringeProcessingImpl;

class ZStereoSystemImpl : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool ready READ ready WRITE setReady NOTIFY readyChanged)

public:
    friend struct ParallelFringeProcessingImpl;

    explicit ZStereoSystemImpl(QObject *parent = 0);
    ~ZStereoSystemImpl();

    bool ready() const;

signals:
    void readyChanged(bool arg);

public slots:
    void stereoRectify(double alpha = -1);

//    cv::Mat getRectifiedSnapshot2D();
    //Z3D::ZSimplePointCloud::Ptr getRectifiedSnapshot3D(int cameraIndex, float imageScale = 50, int lod_step = 4);

    Z3D::ZSimplePointCloud::Ptr triangulateOptimized(const cv::Mat &intensityImg,
            const std::map<int, std::vector<cv::Vec2f> > &leftPoints,
            const std::map<int, std::vector<cv::Vec2f> > &rightPoints,
            int maxPosibleCloudPoints,
            float maxValidDistanceThreshold);

    void setLeftCameraCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration);
    void setRightCameraCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration);

protected slots:
    void precomputeOptimizations();

    inline size_t indexForPixel(int x, int y) const { return size_t(x + y * m_imageSize.width); }

    void setReady(bool arg);

protected:
    std::vector<QPointer<Z3D::ZPinholeCameraCalibration> > mCal;

    std::vector< std::vector<cv::Vec3f> > m_undistortedRays;
    //std::vector< std::vector<cv::Vec3f> > m_undistortedWorldRays;

    cv::Size m_imageSize;

    std::vector<cv::Mat> m_R;
    std::vector<cv::Mat> m_P;
    cv::Mat m_Q;

private:
    bool m_ready;
};

} // namespace Z3D
