#pragma once

#include "zstructuredlightsystem.h"
#include "zcalibratedcamera.h"

namespace Z3D
{

class ZStereoSLSConfigWidget;
class ZStereoSystemImpl;

class ZStereoSLS : public ZStructuredLightSystem
{
    Q_OBJECT

    Q_PROPERTY(double maxValidDistance READ maxValidDistance WRITE setMaxValidDistance NOTIFY maxValidDistanceChanged)

public:
    explicit ZStereoSLS(QObject *parent = 0);
    ~ZStereoSLS();

    virtual QString displayName() override;

    Z3D::ZCalibratedCamera::Ptr leftCamera() const;
    Z3D::ZCalibratedCamera::Ptr rightCamera() const;

    double maxValidDistance() const;

signals:
    void maxValidDistanceChanged(double maxValidDistance);

public slots:
    void setMaxValidDistance(double maxValidDistance);

    // ZStructuredLightSystem interface
    virtual QWidget *configWidget() override;

protected slots:
    void onPatternProjected(ZProjectedPattern::Ptr pattern) override;
    void onPatternsDecoded(std::vector<Z3D::ZDecodedPattern::Ptr> decodedPatterns) override;

private:
    void init();
    void addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras);

    void setLeftCamera(Z3D::ZCalibratedCamera::Ptr camera);
    void setRightCamera(Z3D::ZCalibratedCamera::Ptr camera);

    ZStereoSystemImpl *m_stereoSystem;

    std::vector<Z3D::ZCalibratedCamera::Ptr> m_cameras;

    double m_maxValidDistance;

    /// Config widget (created on demand)
    ZStereoSLSConfigWidget *m_configWidget;
};

} // namespace Z3D
