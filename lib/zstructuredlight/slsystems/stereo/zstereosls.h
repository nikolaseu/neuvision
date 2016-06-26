#pragma once

#include "zstructuredlightsystem.h"
#include "zcameracalibration.h"

namespace Z3D
{

class ZStereoSystemImpl;

class ZStereoSLS : public ZStructuredLightSystem
{
    Q_OBJECT

    Q_PROPERTY(double maxValidDistance READ maxValidDistance WRITE setMaxValidDistance NOTIFY maxValidDistanceChanged)

public:
    explicit ZStereoSLS(QObject *parent = 0);
    ~ZStereoSLS();

    double maxValidDistance() const;

signals:
    void maxValidDistanceChanged(double maxValidDistance);

public slots:
    void setMaxValidDistance(double maxValidDistance);

protected slots:
    void init();

    void setLeftCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration);
    void setRightCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration);

    Z3D::ZSimplePointCloud::Ptr triangulate(const cv::Mat &intensityImg,
            const std::map<int, std::vector<cv::Vec2f> > &leftPoints,
            const std::map<int, std::vector<cv::Vec2f> > &rightPoints,
            int maxPosibleCloudPoints);

private:
    double m_maxValidDistance;
    ZStereoSystemImpl *m_stereoSystem;
};

} // namespace Z3D
