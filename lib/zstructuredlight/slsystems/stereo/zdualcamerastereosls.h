#pragma once

#include "zcalibratedcamera.h"
#include "zstereosls.h"

namespace Z3D
{

class ZDualCameraStereoSLSConfigWidget;

class ZDualCameraStereoSLS : public ZStereoSLS
{
    Q_OBJECT

public:
    explicit ZDualCameraStereoSLS(QObject *parent = 0);
    ~ZDualCameraStereoSLS();

    Z3D::ZCalibratedCamera::Ptr leftCamera() const;
    Z3D::ZCalibratedCamera::Ptr rightCamera() const;

private slots:
    void init();
    void addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras);

    void setLeftCamera(Z3D::ZCalibratedCamera::Ptr camera);
    void setRightCamera(Z3D::ZCalibratedCamera::Ptr camera);

private:
    std::vector<Z3D::ZCalibratedCamera::Ptr> m_cameras;
    ZDualCameraStereoSLSConfigWidget *m_configWidget;

    // ZStructuredLightSystem interface
public:
    virtual QString displayName() override;

public slots:
    virtual QWidget *configWidget() override;

protected slots:
    virtual void onPatternProjected(ZProjectedPattern::Ptr pattern) override;
    virtual void onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> patterns) override;
};

} // namespace Z3D
