#pragma once

#include "zmulticameracalibrator.h"

namespace Z3D
{

class ZOpenCVCustomStereoMultiCameraCalibrator : public ZMultiCameraCalibrator
{
    Q_OBJECT

    Q_PROPERTY(bool fixIntrinsic READ fixIntrinsic WRITE setFixIntrinsic NOTIFY fixIntrinsicChanged)

public:
    explicit ZOpenCVCustomStereoMultiCameraCalibrator(QObject *parent = 0);

    virtual ~ZOpenCVCustomStereoMultiCameraCalibrator();

    // ZMultiCameraCalibrator interface
public:
    virtual QString name();

    virtual std::vector<ZCameraCalibration::Ptr> getCalibration(
            std::vector<ZCameraCalibration::Ptr> &initialCameraCalibrations,
            std::vector<std::vector<std::vector<cv::Point2f> > > &imagePoints,
            std::vector<std::vector<std::vector<cv::Point3f> > > &objectPoints);

    virtual QWidget *configWidget();

public:
    bool fixIntrinsic() const;

public slots:
    void setFixIntrinsic(bool arg);

signals:
    void fixIntrinsicChanged(bool arg);

protected:
    bool m_fixIntrinsic;

    /// configuration widget
    QWidget *m_configWidget;
};

} // namespace Z3D
