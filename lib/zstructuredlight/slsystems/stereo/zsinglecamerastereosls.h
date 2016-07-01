#pragma once

#include "zcalibratedcamera.h"
#include "zstereosls.h"

namespace Z3D
{

class ZSingleCameraStereoSLS : public ZStereoSLS
{
    Q_OBJECT

public:
    explicit ZSingleCameraStereoSLS(QObject *parent = 0);
    ~ZSingleCameraStereoSLS();

private:
    void processPatterns();

    ZCalibratedCamera::Ptr camera;
    ZCameraCalibration::Ptr projectorCalibration;

    ZProjectedPattern::Ptr projectedPattern;
    std::vector<ZDecodedPattern::Ptr> decodedPatterns;

    // ZStructuredLightSystem interface
public:
    virtual QString id() const override;
    virtual QString displayName() const override;

    virtual void init(QSettings *settings) override;

public slots:
    virtual QWidget *configWidget() override;

protected slots:
    virtual void onPatternProjected(ZProjectedPattern::Ptr pattern) override;
    virtual void onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> patterns) override;
};

} // namespace Z3D
