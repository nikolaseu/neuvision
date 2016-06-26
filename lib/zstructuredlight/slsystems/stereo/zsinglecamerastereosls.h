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
