#include "zstereosls.h"

#include "zstereosystemimpl.h"

namespace Z3D
{

ZStereoSLS::ZStereoSLS(QObject *parent)
    : ZStructuredLightSystem(parent)
    , m_maxValidDistance(0.001)
    , m_stereoSystem(nullptr)
{

}

ZStereoSLS::~ZStereoSLS()
{
    qDebug() << "deleting stereo system...";
    if (m_stereoSystem) {
        delete m_stereoSystem;
    }
}

double ZStereoSLS::maxValidDistance() const
{
    return m_maxValidDistance;
}

void ZStereoSLS::setMaxValidDistance(double maxValidDistance)
{
    if (std::fabs(m_maxValidDistance - maxValidDistance) < DBL_EPSILON) {
        return;
    }

    m_maxValidDistance = maxValidDistance;
    emit maxValidDistanceChanged(maxValidDistance);
}

void ZStereoSLS::init()
{
    m_stereoSystem = new ZStereoSystemImpl();

    connect(m_stereoSystem, &ZStereoSystemImpl::readyChanged,
            this, &ZStereoSLS::setReady);
}

void ZStereoSLS::setLeftCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration)
{
    m_stereoSystem->setLeftCameraCalibration(cameraCalibration);
}

void ZStereoSLS::setRightCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration)
{
    m_stereoSystem->setRightCameraCalibration(cameraCalibration);
}

ZSimplePointCloud::Ptr ZStereoSLS::triangulate(const cv::Mat &intensityImg,
                                               const std::map<int, std::vector<cv::Vec2f> > &leftPoints,
                                               const std::map<int, std::vector<cv::Vec2f> > &rightPoints,
                                               int maxPosibleCloudPoints)
{
    if (!m_stereoSystem) {
        return Z3D::ZSimplePointCloud::Ptr(nullptr);
    }

    return m_stereoSystem->triangulateOptimized(intensityImg, leftPoints, rightPoints, maxPosibleCloudPoints, m_maxValidDistance);
}

} // namespace Z3D
